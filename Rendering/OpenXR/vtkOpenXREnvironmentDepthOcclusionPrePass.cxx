// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
#include "vtkOpenXREnvironmentDepthOcclusionPrePass.h"

// OpenGL function pointers (loaded by VTK's GLAD-based loader).
#include "vtk_glad.h"

// XR types: XrEnvironmentDepthImageViewMETA, XrPosef, XrFovf.
// These are available from OpenXR SDK >= 1.1.36 via the platform header.
#include "vtkOpenXRPlatform.h"

#include "vtkOpenXRManager.h" // for GetViewPose / GetProjectionFov

#include "vtkCamera.h"
#include "vtkMatrix4x4.h"
#include "vtkNew.h"
#include "vtkVRHMDCamera.h" // for GetPhysicalToProjectionMatrix()

#include <cmath>    // std::tan
#include <iostream> // std::cerr (shader compile diagnostics)

VTK_ABI_NAMESPACE_BEGIN

//------------------------------------------------------------------------------
// Utility: convert XrPosef → column-major float[16] (pose-in-world, i.e. camera→world).
// Output layout: mat[col*4+row].
static void XrPoseToMatrix(const XrPosef& pose, float mat[16])
{
  const float x = pose.orientation.x;
  const float y = pose.orientation.y;
  const float z = pose.orientation.z;
  const float w = pose.orientation.w;

  // Column 0  (rotation, world-x expressed in local frame)
  mat[0] = 1.0f - 2.0f * (y * y + z * z);
  mat[1] = 2.0f * (x * y + w * z);
  mat[2] = 2.0f * (x * z - w * y);
  mat[3] = 0.0f;
  // Column 1
  mat[4] = 2.0f * (x * y - w * z);
  mat[5] = 1.0f - 2.0f * (x * x + z * z);
  mat[6] = 2.0f * (y * z + w * x);
  mat[7] = 0.0f;
  // Column 2
  mat[8] = 2.0f * (x * z + w * y);
  mat[9] = 2.0f * (y * z - w * x);
  mat[10] = 1.0f - 2.0f * (x * x + y * y);
  mat[11] = 0.0f;
  // Column 3 (translation)
  mat[12] = pose.position.x;
  mat[13] = pose.position.y;
  mat[14] = pose.position.z;
  mat[15] = 1.0f;
}

//------------------------------------------------------------------------------
// Utility: invert a column-major rigid-body (rotation+translation) float[16].
// For M = [R | t; 0 1]:  M^-1 = [R^T | -R^T*t; 0 1].
static void InvertRigidBody(const float m[16], float out[16])
{
  // Transpose the rotation block (upper-left 3×3).
  // m[col*4+row] → out[col*4+row] = m[row*4+col]
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      out[c * 4 + r] = m[r * 4 + c];

  // Compute -R^T * t  (translation in column 3)
  for (int r = 0; r < 3; ++r)
  {
    float s = 0.0f;
    for (int k = 0; k < 3; ++k)
      s += m[k * 4 + r] * m[12 + k]; // R^T[r][k] = m[k*4+r]; t[k] = m[12+k]
    out[12 + r] = -s;
  }

  out[3] = out[7] = out[11] = 0.0f;
  out[15] = 1.0f;
}

//------------------------------------------------------------------------------
// Utility: copy a VTK (row-major, double) matrix to a GL (column-major, float) array.
static void VtkMatrixToGL(vtkMatrix4x4* m, float out[16])
{
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      out[col * 4 + row] = static_cast<float>(m->GetElement(row, col));
}

//------------------------------------------------------------------------------
// Utility: build a column-major GL projection matrix from an OpenXR asymmetric
// FOV + near/far clipping planes.
// Equivalent to glFrustum but using tangent angles directly.
static void BuildProjectionFromFov(const XrFovf& fov, float nearZ, float farZ, float out[16])
{
  const float l = std::tan(fov.angleLeft);   // negative
  const float r = std::tan(fov.angleRight);  // positive
  const float b = std::tan(fov.angleDown);   // negative
  const float t = std::tan(fov.angleUp);     // positive
  const float n = nearZ, f = farZ;

  // Column-major layout: out[col*4 + row]
  // Column 0
  out[0] = 2.0f / (r - l); out[1] = 0.0f;            out[2] = 0.0f;                out[3] = 0.0f;
  // Column 1
  out[4] = 0.0f;            out[5] = 2.0f / (t - b);  out[6] = 0.0f;                out[7] = 0.0f;
  // Column 2
  out[8]  = (r + l) / (r - l);
  out[9]  = (t + b) / (t - b);
  out[10] = -(f + n) / (f - n);
  out[11] = -1.0f;
  // Column 3
  out[12] = 0.0f; out[13] = 0.0f; out[14] = -2.0f * f * n / (f - n); out[15] = 0.0f;
}

//------------------------------------------------------------------------------
// Compile a single shader stage; returns 0 on failure.
static GLuint CompileShader(GLenum type, const char* src)
{
  GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);

  GLint ok = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok)
  {
    char log[1024] = {};
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    std::cerr << "vtkOpenXREnvironmentDepthOcclusionPrePass: shader compile error:\n"
              << log << "\n";
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

//------------------------------------------------------------------------------
// Fullscreen triangle vertex shader.
// Three vertices (gl_VertexID 0,1,2) produce a CCW triangle that covers [-1,1]×[-1,1].
static const char* s_VertSrc = R"glsl(
#version 330 core
out vec2 vTexCoord;
void main()
{
    // Vertex 0: (-1,-1)  Vertex 1: (3,-1)  Vertex 2: (-1,3)
    // Together they cover the full NDC square via gl_VertexID.
    float x = float((gl_VertexID & 1) << 2) - 1.0;
    float y = float((gl_VertexID >> 1 & 1) << 2) - 1.0;
    gl_Position = vec4(x, y, 0.0, 1.0);
    vTexCoord   = gl_Position.xy * 0.5 + 0.5;
}
)glsl";

//------------------------------------------------------------------------------
// Environment-depth occlusion fragment shader.
// For each screen pixel it:
//   1. Unprojects the pixel into world-space ray direction (via VR proj/view).
//   2. Reprojects that ray into the env-depth camera's local space.
//   3. Samples the real-world radial depth using a tangent-based FOV mapping.
//   4. Reconstructs the world-space surface point and re-projects to VR clip space.
//   5. Writes gl_FragDepth (colour writes are disabled by Apply()).
//
// NOTE: vrView / envDepthViewInverse must share the same world-space coordinate
// frame.  When VTK's PhysicalToWorldMatrix is identity (the common case), the
// OpenXR reference-space pose is identical to the VTK world space and no
// additional transform is required.
static const char* s_FragSrc = R"glsl(
#version 330 core

uniform sampler2DArray envDepthTex;   // GL_TEXTURE_2D_ARRAY; layer = eye index

// VR eye matrices
uniform mat4 vrProjInverse;           // inverse of VR eye projection
uniform mat4 vrProj;                  // VR eye projection (eye → clip)
uniform mat4 vrView;                  // world → VR eye space
uniform mat4 vrViewInverse;           // VR eye → world (eye pose in world)

// Environment-depth camera matrices (derived from XrPosef)
uniform mat4 envDepthViewInverse;     // env-depth camera pose in world (col3 = cam pos)
uniform mat4 envDepthView;            // world → env-depth camera space

// Precomputed tangents of the env-depth FOV angles: tan(left), tan(right), tan(down), tan(up).
// left/down are negative; right/up are positive.
uniform vec4 envDepthTan;

uniform int eye;                      // 0 = left, 1 = right

in vec2 vTexCoord;

void main()
{
    // -----------------------------------------------------------------------
    // 1. NDC pixel → eye-space position/direction
    // -----------------------------------------------------------------------
    vec2 ndc = vTexCoord * 2.0 - 1.0;
    vec4 eyeH = vrProjInverse * vec4(ndc, 0.0, 1.0);
    eyeH /= eyeH.w;
    vec3 eyeDir = normalize(eyeH.xyz);

    // -----------------------------------------------------------------------
    // 2. Eye-space direction → world-space direction
    // -----------------------------------------------------------------------
    vec3 worldDir = normalize(mat3(vrViewInverse) * eyeDir);

    // -----------------------------------------------------------------------
    // 3. World-space direction → env-depth camera space
    // -----------------------------------------------------------------------
    vec3 edDir = normalize(mat3(envDepthView) * worldDir);

    // In OpenXR (and OpenGL/VTK), cameras look along the NEGATIVE Z axis in
    // their local frame.  A ray pointing INTO the camera frustum therefore has
    // a NEGATIVE z component in env-depth camera space.  Discard any ray that
    // points away from (behind) the camera, i.e. z >= 0.
    if (edDir.z >= 0.0)
    {
        gl_FragDepth = 1.0;
        return;
    }

    // -----------------------------------------------------------------------
    // 4. Compute UV from tangent-based asymmetric FOV
    // From the OpenXR spec: fov.angleLeft < 0, fov.angleRight > 0,
    //                        fov.angleDown < 0, fov.angleUp   > 0.
    // Divide by -edDir.z (positive for forward rays) to get the tangent of
    // the angle from the optical axis.
    // -----------------------------------------------------------------------
    float tanLeft  = envDepthTan.x;
    float tanRight = envDepthTan.y;
    float tanDown  = envDepthTan.z;
    float tanUp    = envDepthTan.w;

    float u = (edDir.x / (-edDir.z) - tanLeft)  / (tanRight - tanLeft);
    float v = (edDir.y / (-edDir.z) - tanDown)  / (tanUp    - tanDown);

    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0)
    {
        gl_FragDepth = 1.0;
        return;
    }

    // -----------------------------------------------------------------------
    // 5. Sample real-world radial depth (metres along the ray)
    // -----------------------------------------------------------------------
    float realDepth = texture(envDepthTex, vec3(u, v, float(eye))).r;
    if (realDepth <= 0.0)
    {
        gl_FragDepth = 1.0;
        return;
    }

    // -----------------------------------------------------------------------
    // 6. Reconstruct world-space surface point
    //    realDepth is radial (Euclidean distance along the ray from the
    //    env-depth camera, NOT projected z).
    // -----------------------------------------------------------------------
    vec3 edCamPos    = envDepthViewInverse[3].xyz;
    vec3 edWorldDir  = normalize(mat3(envDepthViewInverse) * edDir);
    vec3 surfaceWorld = edCamPos + edWorldDir * realDepth;

    // -----------------------------------------------------------------------
    // 7. Re-project world-space point into VR clip space → write depth
    // -----------------------------------------------------------------------
    vec4 vrClip = vrProj * (vrView * vec4(surfaceWorld, 1.0));
    if (vrClip.w <= 0.0) { gl_FragDepth = 1.0; return; }
    gl_FragDepth = (vrClip.z / vrClip.w) * 0.5 + 0.5;
}
)glsl";

//------------------------------------------------------------------------------
// Debug visualisation fragment shader.
// Uses identical reprojection to s_FragSrc but outputs a false-colour depth
// image as an alpha-blended colour overlay instead of writing gl_FragDepth.
//
//   red   = near  (0 m)
//   blue  = far   (~5 m, controlled by uniform depthScale)
//   black = outside env-depth frustum or invalid pixel
//   alpha = 0.75  (semi-transparent so the underlying scene shows through)
//
static const char* s_DebugFragSrc = R"glsl(
#version 330 core

uniform sampler2DArray envDepthTex;
uniform mat4 vrProjInverse;
uniform mat4 vrView;
uniform mat4 vrViewInverse;
uniform mat4 envDepthViewInverse;
uniform mat4 envDepthView;
uniform vec4 envDepthTan;   // precomputed tan() of FOV angles: left, right, down, up
uniform int  eye;
uniform float depthScale;   // metres at which the colour reaches full blue (default 5.0)

in  vec2 vTexCoord;
out vec4 fragColor;

vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

void main()
{
    // 1. NDC → eye-space direction (reuse vrProjInverse)
    vec2 ndc  = vTexCoord * 2.0 - 1.0;
    vec4 eyeH = vrProjInverse * vec4(ndc, 0.0, 1.0);
    eyeH /= eyeH.w;
    vec3 eyeDir = normalize(eyeH.xyz);

    // 2. Eye → world
    vec3 worldDir = normalize(mat3(vrViewInverse) * eyeDir);

    // 3. World → env-depth camera space
    vec3 edDir = normalize(mat3(envDepthView) * worldDir);

    // OpenXR -Z-forward: forward rays have edDir.z < 0
    if (edDir.z >= 0.0)
    {
        discard;
    }

    // 4. Tangent-based UV
    float tanLeft  = envDepthTan.x;
    float tanRight = envDepthTan.y;
    float tanDown  = envDepthTan.z;
    float tanUp    = envDepthTan.w;

    float u = (edDir.x / (-edDir.z) - tanLeft) / (tanRight - tanLeft);
    float v = (edDir.y / (-edDir.z) - tanDown) / (tanUp    - tanDown);

    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0)
    {
        discard;
    }

    // 5. Sample real-world depth
    float realDepth = texture(envDepthTex, vec3(u, v, float(eye))).r;
    if (realDepth <= 0.0)
    {
        discard;
    }

    // 6. Map depth → false colour
    float t = clamp(realDepth / depthScale, 0.0, 1.0);
    vec3 color = hsv2rgb(vec3(t, 1.0, 1.0));

    // alpha = 0.75: semi-transparent so the underlying scene shows through
    fragColor = vec4(color, 0.75);
}
)glsl";

//------------------------------------------------------------------------------
// Partial occlusion post-pass fragment shader.
// Same reprojection as s_FragSrc but uses gl_FragDepth for the depth TEST
// (GL_LESS, no depth WRITE) to detect pixels where virtual geometry is behind
// the real world, and outputs an alpha value that, combined with the caller's
// blend state, multiplies the existing colour alpha by occludedOpacity.
//
// Blend setup used by the caller:
//   glBlendFuncSeparate(GL_ZERO, GL_ONE, GL_ZERO, GL_ONE_MINUS_SRC_ALPHA)
//   glColorMask(false, false, false, true)  — alpha channel only
//
// This fragment outputs src.a = (1 - occludedOpacity).
// Effect per fragment that passes the depth test (GL_ONE_MINUS_SRC_ALPHA dst factor):
//   result.a = dst.a * (1 - src.a)
//            = dst.a * occludedOpacity
//
// Empty pixels (dst.a = 0) are therefore left at 0; only pixels where the
// scene rendered visible geometry have their alpha scaled down.
//
static const char* s_OcclusionPostPassFragSrc = R"glsl(
#version 330 core

uniform sampler2DArray envDepthTex;
uniform mat4 vrProjInverse;
// physicalToClip = VTK's actual PhysicalToProjectionMatrix for the active eye.
// Using VTK's own matrix (rather than rebuilding from FOV + near/far) guarantees
// that the gl_FragDepth values we write are encoded identically to the values
// already in the scene depth buffer, so the GL_LESS comparison is valid.
uniform mat4 physicalToClip;
uniform mat4 vrViewInverse;
uniform mat4 envDepthViewInverse;
uniform mat4 envDepthView;
uniform vec4 envDepthTan;   // precomputed tan() of FOV angles: left, right, down, up
uniform int  eye;
// occludedOpacity in [0, 1):
//   0.0 = completely occlude (result alpha → 0)
//   0.5 = half transparent   (result alpha → 0.5 * original)
uniform float occludedOpacity;

// Gaussian sigma in texels of the env-depth texture.  Blurs the raw depth
// values before the depth comparison so that low-resolution sensor data does
// not produce a blocky occlusion boundary.
uniform float sigmaTexels;

in  vec2 vTexCoord;
out vec4 fragColor;

void main()
{
    // 1. NDC → eye-space direction
    vec2 ndc  = vTexCoord * 2.0 - 1.0;
    vec4 eyeH = vrProjInverse * vec4(ndc, 0.0, 1.0);
    eyeH /= eyeH.w;
    vec3 eyeDir = normalize(eyeH.xyz);

    // 2. Eye → physical (OpenXR reference) space direction
    vec3 worldDir = normalize(mat3(vrViewInverse) * eyeDir);

    // 3. Physical → env-depth camera space
    vec3 edDir = normalize(mat3(envDepthView) * worldDir);

    // OpenXR -Z-forward: forward rays have edDir.z < 0
    if (edDir.z >= 0.0)
    {
        discard;
    }

    // 4. Tangent-based UV
    float tanLeft  = envDepthTan.x;
    float tanRight = envDepthTan.y;
    float tanDown  = envDepthTan.z;
    float tanUp    = envDepthTan.w;

    float u = (edDir.x / (-edDir.z) - tanLeft) / (tanRight - tanLeft);
    float v = (edDir.y / (-edDir.z) - tanDown) / (tanUp    - tanDown);

    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0)
    {
        discard;
    }

    // 5. Sample real-world depth (metres, euclidean) with a Gaussian blur over
    //    the env-depth texture.  This smooths the raw depth values from the
    //    low-resolution sensor before comparison so that the resulting occlusion
    //    boundary is not limited by the sensor's pixel grid.
    //    Only valid (non-zero) depth texels contribute to the weighted sum,
    //    so that missing/invalid pixels at the sensor boundary do not bias the
    //    depth estimate inward.
    float realDepth;
    {
        ivec3 edSize     = textureSize(envDepthTex, 0);
        vec2  edTexelSize = 1.0 / vec2(edSize.xy);
        float s2         = 2.0 * sigmaTexels * sigmaTexels;
        float depthAcc   = 0.0;
        float wSum       = 0.0;
        for (int dy = -2; dy <= 2; ++dy)
        {
            for (int dx = -2; dx <= 2; ++dx)
            {
                float w   = exp(-float(dx * dx + dy * dy) / s2);
                vec2  uv2 = clamp(vec2(u, v) + vec2(float(dx), float(dy)) * edTexelSize,
                                  0.0, 1.0);
                float d   = texture(envDepthTex, vec3(uv2, float(eye))).r;
                if (d > 0.0) { depthAcc += w * d; wSum += w; }
            }
        }
        if (wSum <= 0.0) { discard; }
        realDepth = depthAcc / wSum;
    }

    // 6. Reconstruct physical-space surface point
    //    realDepth is radial distance from the env-depth camera along the ray.
    vec3 edCamPos    = envDepthViewInverse[3].xyz;
    vec3 edWorldDir  = normalize(mat3(envDepthViewInverse) * edDir);
    vec3 surfaceWorld = edCamPos + edWorldDir * realDepth;

    // 7. Re-project to VR clip space using VTK's own physical→clip matrix.
    //    This ensures gl_FragDepth is on the same scale as the scene depth buffer.
    vec4 vrClip = physicalToClip * vec4(surfaceWorld, 1.0);

    // Guard against degenerate projections (object behind or at camera).
    if (vrClip.w <= 0.0) { discard; }

    // A small positive bias shifts the real-world depth slightly away from
    // the camera so that virtual geometry at exactly the same distance as
    // a real surface is treated as "in front" and left visible.  This
    // prevents false occlusion at the boundary.  1e-4 in NDC corresponds
    // to a sub-centimetre tolerance at typical XR working distances.
    gl_FragDepth = (vrClip.z / vrClip.w) * 0.5 + 0.5 + 1e-4;

    // Depth test passes for occluded pixels.  The caller uses
    // glBlendFuncSeparate(GL_ZERO, GL_ONE, GL_ZERO, GL_ONE_MINUS_SRC_ALPHA)
    // so the result alpha = dst.a * (1 - src.a) = dst.a * occludedOpacity.
    fragColor = vec4(0.0, 0.0, 0.0, 1.0 - occludedOpacity);
}
)glsl";

//------------------------------------------------------------------------------
vtkOpenXREnvironmentDepthOcclusionPrePass::vtkOpenXREnvironmentDepthOcclusionPrePass() = default;

//------------------------------------------------------------------------------
vtkOpenXREnvironmentDepthOcclusionPrePass::~vtkOpenXREnvironmentDepthOcclusionPrePass()
{
  if (this->Program)
  {
    glDeleteProgram(this->Program);
  }
  if (this->DebugProgram)
  {
    glDeleteProgram(this->DebugProgram);
  }
  if (this->OcclusionPostPassProgram)
  {
    glDeleteProgram(this->OcclusionPostPassProgram);
  }
  if (this->VAO)
  {
    glDeleteVertexArrays(1, &this->VAO);
  }
}

//------------------------------------------------------------------------------
void vtkOpenXREnvironmentDepthOcclusionPrePass::BuildProgram()
{
  GLuint vert = CompileShader(GL_VERTEX_SHADER, s_VertSrc);
  GLuint frag = CompileShader(GL_FRAGMENT_SHADER, s_FragSrc);

  if (!vert || !frag)
  {
    glDeleteShader(vert);
    glDeleteShader(frag);
    return;
  }

  GLuint prog = glCreateProgram();
  glAttachShader(prog, vert);
  glAttachShader(prog, frag);
  glLinkProgram(prog);

  glDeleteShader(vert);
  glDeleteShader(frag);

  GLint ok = GL_FALSE;
  glGetProgramiv(prog, GL_LINK_STATUS, &ok);
  if (!ok)
  {
    char log[1024] = {};
    glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
    std::cerr << "vtkOpenXREnvironmentDepthOcclusionPrePass: program link error:\n"
              << log << "\n";
    glDeleteProgram(prog);
    return;
  }

  this->Program = static_cast<uint32_t>(prog);

  glGenVertexArrays(1, &this->VAO);
}

//------------------------------------------------------------------------------
void vtkOpenXREnvironmentDepthOcclusionPrePass::Apply(uint32_t eye, vtkCamera* camera,
  uint32_t envDepthGLTexture, const XrEnvironmentDepthImageViewMETA views[2],
  float physicalScale)
{
  if (!camera)
  {
    return;
  }

  // Lazy-initialise the GLSL program on first use.
  if (!this->Program)
  {
    this->BuildProgram();
    if (!this->Program)
    {
      return; // shader compilation failed — silently skip
    }
  }

  // ------------------------------------------------------------------
  // Compute uniforms from camera and pose data.
  // ------------------------------------------------------------------

  // Clipping planes — independent of which eye is active on the camera.
  // camera->GetClippingRange() returns values in VTK world units.  The env-depth
  // shader works in OpenXR metres, which are related by PhysicalScale:
  //   vtk_world_unit = openxr_metre * PhysicalScale
  // Dividing near/far by PhysicalScale converts them to metres so that the
  // projection matrix built here matches the depth values the shader produces.
  double clippingRange[2];
  camera->GetClippingRange(clippingRange);
  const float nearZ = static_cast<float>(clippingRange[0]) / physicalScale;
  const float farZ  = static_cast<float>(clippingRange[1]) / physicalScale;

  // VR eye matrices: derive directly from OpenXR view data so that the
  // result is correct regardless of whether VTK has called SetLeftEye()
  // yet for this eye.
  vtkOpenXRManager& mgr = vtkOpenXRManager::GetInstance();
  const XrPosef* vrEyePosePtr = mgr.GetViewPose(eye);
  const XrFovf*  vrFovPtr     = mgr.GetProjectionFov(eye);
  if (!vrEyePosePtr || !vrFovPtr)
  {
    return; // session not started or view data not initialised
  }

  // Build VR projection from FOV + near/far.
  float vrProj[16];
  BuildProjectionFromFov(*vrFovPtr, nearZ, farZ, vrProj);

  // Invert the projection matrix for use in the unproject step.
  float vrProjInverse[16];
  {
    vtkNew<vtkMatrix4x4> projMat;
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        projMat->SetElement(row, col, static_cast<double>(vrProj[col * 4 + row]));
    vtkNew<vtkMatrix4x4> projInvMat;
    vtkMatrix4x4::Invert(projMat, projInvMat);
    VtkMatrixToGL(projInvMat, vrProjInverse);
  }

  // Build VR view matrix from the eye pose (OpenXR reference space).
  float vrViewInverse[16]; // eye-in-world (pose → column-major matrix)
  XrPoseToMatrix(*vrEyePosePtr, vrViewInverse);
  float vrView[16];        // world-to-eye
  InvertRigidBody(vrViewInverse, vrView);

  // Environment-depth camera pose (world → envDepth camera).
  // XrPosef is in the OpenXR reference space, which coincides with VTK world
  // space when PhysicalToWorldMatrix is identity.
  const XrEnvironmentDepthImageViewMETA& view = views[eye];
  float envDepthViewInverse[16]; // pose (XrPosef) → camera-in-world
  XrPoseToMatrix(view.pose, envDepthViewInverse);
  float envDepthViewMat[16]; // world → env-depth camera space
  InvertRigidBody(envDepthViewInverse, envDepthViewMat);

  // Precomputed FOV tangents (avoids per-fragment tan() in the shader).
  const XrFovf& fov = view.fov;
  float envDepthTan[4] = {
    std::tan(fov.angleLeft),  // x  (negative)
    std::tan(fov.angleRight), // y  (positive)
    std::tan(fov.angleDown),  // z  (negative)
    std::tan(fov.angleUp),    // w  (positive)
  };

  // ------------------------------------------------------------------
  // Save GL state.
  // ------------------------------------------------------------------
  GLboolean colorMask[4];
  glGetBooleanv(GL_COLOR_WRITEMASK, colorMask);
  GLboolean depthMask;
  glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
  GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
  GLint depthFunc;
  glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);
  GLint prevProgram;
  glGetIntegerv(GL_CURRENT_PROGRAM, &prevProgram);
  GLint prevActiveTexUnit;
  glGetIntegerv(GL_ACTIVE_TEXTURE, &prevActiveTexUnit);
  GLint prevTexBinding;
  glActiveTexture(GL_TEXTURE0);
  glGetIntegerv(GL_TEXTURE_BINDING_2D_ARRAY, &prevTexBinding);
  GLint prevVAO;
  glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &prevVAO);

  // ------------------------------------------------------------------
  // Configure GL for depth-only pre-pass.
  // ------------------------------------------------------------------
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE); // no colour writes
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_ALWAYS); // unconditionally write all depths

  // ------------------------------------------------------------------
  // Draw the fullscreen triangle with the depth shader.
  // ------------------------------------------------------------------
  glUseProgram(static_cast<GLuint>(this->Program));

  // Bind textures.
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D_ARRAY, static_cast<GLuint>(envDepthGLTexture));
  glUniform1i(glGetUniformLocation(this->Program, "envDepthTex"), 0);

  // Upload matrices and scalars.
  glUniformMatrix4fv(
    glGetUniformLocation(this->Program, "vrProjInverse"), 1, GL_FALSE, vrProjInverse);
  glUniformMatrix4fv(glGetUniformLocation(this->Program, "vrProj"), 1, GL_FALSE, vrProj);
  glUniformMatrix4fv(glGetUniformLocation(this->Program, "vrView"), 1, GL_FALSE, vrView);
  glUniformMatrix4fv(
    glGetUniformLocation(this->Program, "vrViewInverse"), 1, GL_FALSE, vrViewInverse);
  glUniformMatrix4fv(glGetUniformLocation(this->Program, "envDepthViewInverse"), 1, GL_FALSE,
    envDepthViewInverse);
  glUniformMatrix4fv(
    glGetUniformLocation(this->Program, "envDepthView"), 1, GL_FALSE, envDepthViewMat);
  glUniform4fv(glGetUniformLocation(this->Program, "envDepthTan"), 1, envDepthTan);
  glUniform1i(glGetUniformLocation(this->Program, "eye"), static_cast<GLint>(eye));

  glBindVertexArray(static_cast<GLuint>(this->VAO));
  glDrawArrays(GL_TRIANGLES, 0, 3);

  // ------------------------------------------------------------------
  // Restore GL state.
  // ------------------------------------------------------------------
  glBindVertexArray(static_cast<GLuint>(prevVAO));
  glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
  glDepthMask(depthMask);
  glDepthFunc(static_cast<GLenum>(depthFunc));
  if (!depthTestEnabled)
  {
    glDisable(GL_DEPTH_TEST);
  }
  glUseProgram(static_cast<GLuint>(prevProgram));
  glBindTexture(GL_TEXTURE_2D_ARRAY, static_cast<GLuint>(prevTexBinding));
  glActiveTexture(static_cast<GLenum>(prevActiveTexUnit));
}

//------------------------------------------------------------------------------
void vtkOpenXREnvironmentDepthOcclusionPrePass::BuildDebugProgram()
{
  GLuint vert = CompileShader(GL_VERTEX_SHADER, s_VertSrc);
  GLuint frag = CompileShader(GL_FRAGMENT_SHADER, s_DebugFragSrc);

  if (!vert || !frag)
  {
    glDeleteShader(vert);
    glDeleteShader(frag);
    return;
  }

  GLuint prog = glCreateProgram();
  glAttachShader(prog, vert);
  glAttachShader(prog, frag);
  glLinkProgram(prog);

  glDeleteShader(vert);
  glDeleteShader(frag);

  GLint ok = GL_FALSE;
  glGetProgramiv(prog, GL_LINK_STATUS, &ok);
  if (!ok)
  {
    char log[1024] = {};
    glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
    std::cerr << "vtkOpenXREnvironmentDepthOcclusionPrePass: debug program link error:\n"
              << log << "\n";
    glDeleteProgram(prog);
    return;
  }

  this->DebugProgram = static_cast<uint32_t>(prog);

  // Re-use the VAO already created by BuildProgram(); create it here if this
  // method is somehow called standalone.
  if (!this->VAO)
  {
    glGenVertexArrays(1, &this->VAO);
  }
}

//------------------------------------------------------------------------------
void vtkOpenXREnvironmentDepthOcclusionPrePass::DebugVisualize(uint32_t eye, vtkCamera* camera,
  uint32_t envDepthGLTexture, const XrEnvironmentDepthImageViewMETA views[2])
{
  if (!camera)
  {
    return;
  }

  // Ensure the shared VAO exists (may have been created by BuildProgram already).
  if (!this->DebugProgram)
  {
    // Also builds the VAO if not yet present.
    if (!this->VAO)
    {
      glGenVertexArrays(1, &this->VAO);
    }
    this->BuildDebugProgram();
    if (!this->DebugProgram)
    {
      return;
    }
  }

  // ------------------------------------------------------------------
  // Build the same uniforms as Apply().
  // ------------------------------------------------------------------
  double clippingRange[2];
  camera->GetClippingRange(clippingRange);
  const float nearZ = static_cast<float>(clippingRange[0]);
  const float farZ  = static_cast<float>(clippingRange[1]);

  vtkOpenXRManager& mgr = vtkOpenXRManager::GetInstance();
  const XrPosef* vrEyePosePtr = mgr.GetViewPose(eye);
  const XrFovf*  vrFovPtr     = mgr.GetProjectionFov(eye);
  if (!vrEyePosePtr || !vrFovPtr)
  {
    return;
  }

  float vrProj[16];
  BuildProjectionFromFov(*vrFovPtr, nearZ, farZ, vrProj);

  float vrProjInverse[16];
  {
    vtkNew<vtkMatrix4x4> projMat;
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        projMat->SetElement(row, col, static_cast<double>(vrProj[col * 4 + row]));
    vtkNew<vtkMatrix4x4> projInvMat;
    vtkMatrix4x4::Invert(projMat, projInvMat);
    VtkMatrixToGL(projInvMat, vrProjInverse);
  }

  float vrViewInverse[16];
  XrPoseToMatrix(*vrEyePosePtr, vrViewInverse);
  float vrView[16];
  InvertRigidBody(vrViewInverse, vrView);

  const XrEnvironmentDepthImageViewMETA& view = views[eye];
  float envDepthViewInverse[16];
  XrPoseToMatrix(view.pose, envDepthViewInverse);
  float envDepthViewMat[16];
  InvertRigidBody(envDepthViewInverse, envDepthViewMat);

  const XrFovf& fov = view.fov;
  float envDepthTan[4] = { std::tan(fov.angleLeft), std::tan(fov.angleRight),
    std::tan(fov.angleDown), std::tan(fov.angleUp) };

  // ------------------------------------------------------------------
  // Save GL state.
  // ------------------------------------------------------------------
  GLboolean colorMask[4];
  glGetBooleanv(GL_COLOR_WRITEMASK, colorMask);
  GLboolean depthMask;
  glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
  GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
  GLboolean blendEnabled = glIsEnabled(GL_BLEND);
  GLint blendSrcRGB, blendDstRGB, blendSrcAlpha, blendDstAlpha;
  glGetIntegerv(GL_BLEND_SRC_RGB, &blendSrcRGB);
  glGetIntegerv(GL_BLEND_DST_RGB, &blendDstRGB);
  glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendSrcAlpha);
  glGetIntegerv(GL_BLEND_DST_ALPHA, &blendDstAlpha);
  GLint prevProgram;
  glGetIntegerv(GL_CURRENT_PROGRAM, &prevProgram);
  GLint prevActiveTexUnit;
  glGetIntegerv(GL_ACTIVE_TEXTURE, &prevActiveTexUnit);
  GLint prevTexBinding;
  glActiveTexture(GL_TEXTURE0);
  glGetIntegerv(GL_TEXTURE_BINDING_2D_ARRAY, &prevTexBinding);
  GLint prevVAO;
  glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &prevVAO);

  // ------------------------------------------------------------------
  // Configure GL: colour overlay on top of scene, no depth writes.
  // ------------------------------------------------------------------
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glDisable(GL_DEPTH_TEST); // overlay on top of everything
  glDepthMask(GL_FALSE);    // do not disturb the depth buffer
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // ------------------------------------------------------------------
  // Draw.
  // ------------------------------------------------------------------
  glUseProgram(static_cast<GLuint>(this->DebugProgram));

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D_ARRAY, static_cast<GLuint>(envDepthGLTexture));
  glUniform1i(glGetUniformLocation(this->DebugProgram, "envDepthTex"), 0);

  glUniformMatrix4fv(
    glGetUniformLocation(this->DebugProgram, "vrProjInverse"), 1, GL_FALSE, vrProjInverse);
  glUniformMatrix4fv(
    glGetUniformLocation(this->DebugProgram, "vrView"), 1, GL_FALSE, vrView);
  glUniformMatrix4fv(
    glGetUniformLocation(this->DebugProgram, "vrViewInverse"), 1, GL_FALSE, vrViewInverse);
  glUniformMatrix4fv(glGetUniformLocation(this->DebugProgram, "envDepthViewInverse"), 1, GL_FALSE,
    envDepthViewInverse);
  glUniformMatrix4fv(
    glGetUniformLocation(this->DebugProgram, "envDepthView"), 1, GL_FALSE, envDepthViewMat);
  glUniform4fv(glGetUniformLocation(this->DebugProgram, "envDepthTan"), 1, envDepthTan);
  glUniform1i(glGetUniformLocation(this->DebugProgram, "eye"), static_cast<GLint>(eye));
  glUniform1f(glGetUniformLocation(this->DebugProgram, "depthScale"), 5.0f);

  glBindVertexArray(static_cast<GLuint>(this->VAO));
  glDrawArrays(GL_TRIANGLES, 0, 3);

  // ------------------------------------------------------------------
  // Restore GL state.
  // ------------------------------------------------------------------
  glBindVertexArray(static_cast<GLuint>(prevVAO));
  glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
  glDepthMask(depthMask);
  if (depthTestEnabled)
  {
    glEnable(GL_DEPTH_TEST);
  }
  glBlendFuncSeparate(static_cast<GLenum>(blendSrcRGB), static_cast<GLenum>(blendDstRGB),
    static_cast<GLenum>(blendSrcAlpha), static_cast<GLenum>(blendDstAlpha));
  if (!blendEnabled)
  {
    glDisable(GL_BLEND);
  }
  glUseProgram(static_cast<GLuint>(prevProgram));
  glBindTexture(GL_TEXTURE_2D_ARRAY, static_cast<GLuint>(prevTexBinding));
  glActiveTexture(static_cast<GLenum>(prevActiveTexUnit));
}

//------------------------------------------------------------------------------
void vtkOpenXREnvironmentDepthOcclusionPrePass::BuildOcclusionPostPassProgram()
{
  GLuint vert = CompileShader(GL_VERTEX_SHADER, s_VertSrc);
  GLuint frag = CompileShader(GL_FRAGMENT_SHADER, s_OcclusionPostPassFragSrc);

  if (!vert || !frag)
  {
    glDeleteShader(vert);
    glDeleteShader(frag);
    return;
  }

  GLuint prog = glCreateProgram();
  glAttachShader(prog, vert);
  glAttachShader(prog, frag);
  glLinkProgram(prog);

  glDeleteShader(vert);
  glDeleteShader(frag);

  GLint ok = GL_FALSE;
  glGetProgramiv(prog, GL_LINK_STATUS, &ok);
  if (!ok)
  {
    char log[1024] = {};
    glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
    std::cerr << "vtkOpenXREnvironmentDepthOcclusionPrePass: occlusion post-pass link error:\n"
              << log << "\n";
    glDeleteProgram(prog);
    return;
  }

  this->OcclusionPostPassProgram = static_cast<uint32_t>(prog);

  if (!this->VAO)
  {
    glGenVertexArrays(1, &this->VAO);
  }
}

//------------------------------------------------------------------------------
void vtkOpenXREnvironmentDepthOcclusionPrePass::ApplyOcclusionPostPass(uint32_t eye,
  vtkCamera* camera, uint32_t envDepthGLTexture, const XrEnvironmentDepthImageViewMETA views[2],
  float occludedOpacity, float physicalScale, float sigmaTexels)
{
  if (!camera || occludedOpacity >= 1.0f)
  {
    return; // nothing to do
  }

  if (!this->OcclusionPostPassProgram)
  {
    if (!this->VAO)
    {
      glGenVertexArrays(1, &this->VAO);
    }
    this->BuildOcclusionPostPassProgram();
    if (!this->OcclusionPostPassProgram)
    {
      return;
    }
  }

  // ------------------------------------------------------------------
  // Compute uniforms.
  // ------------------------------------------------------------------
  // physicalToClip: use VTK's own PhysicalToProjectionMatrix so that the
  // gl_FragDepth values we emit are encoded on exactly the same scale as
  // the depths already in the scene depth buffer.  Building this from FOV +
  // clippingRange/physicalScale is unreliable because VTK may auto-adjust
  // clipping planes during rendering, making the near/far values differ.
  float physicalToClip[16];
  {
    vtkVRHMDCamera* vrCam = vtkVRHMDCamera::SafeDownCast(camera);
    if (!vrCam)
    {
      return; // not a VR camera — can't obtain the correct matrix
    }
    vtkMatrix4x4* mat = nullptr;
    vrCam->GetPhysicalToProjectionMatrix(mat);
    if (!mat)
    {
      return;
    }
    VtkMatrixToGL(mat, physicalToClip);
  }

  // vrProjInverse: used only to unproject screen NDC → eye-space ray direction
  // (step 1 in the shader).  A small inaccuracy here just shifts WHERE we
  // sample the env-depth texture, not the depth comparison result.
  vtkOpenXRManager& mgr = vtkOpenXRManager::GetInstance();
  const XrPosef* vrEyePosePtr = mgr.GetViewPose(eye);
  const XrFovf*  vrFovPtr     = mgr.GetProjectionFov(eye);
  if (!vrEyePosePtr || !vrFovPtr)
  {
    return;
  }

  double clippingRange[2];
  camera->GetClippingRange(clippingRange);
  const float nearZ = static_cast<float>(clippingRange[0]) / physicalScale;
  const float farZ  = static_cast<float>(clippingRange[1]) / physicalScale;

  float vrProjForInverse[16];
  BuildProjectionFromFov(*vrFovPtr, nearZ, farZ, vrProjForInverse);

  float vrProjInverse[16];
  {
    vtkNew<vtkMatrix4x4> projMat;
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        projMat->SetElement(row, col, static_cast<double>(vrProjForInverse[col * 4 + row]));
    vtkNew<vtkMatrix4x4> projInvMat;
    vtkMatrix4x4::Invert(projMat, projInvMat);
    VtkMatrixToGL(projInvMat, vrProjInverse);
  }

  // vrViewInverse: eye pose in OpenXR physical space.
  // Used only for transforming eye-space ray directions to physical space
  // (step 2 in the shader).
  float vrViewInverse[16];
  XrPoseToMatrix(*vrEyePosePtr, vrViewInverse);

  const XrEnvironmentDepthImageViewMETA& view = views[eye];
  float envDepthViewInverse[16];
  XrPoseToMatrix(view.pose, envDepthViewInverse);
  float envDepthViewMat[16];
  InvertRigidBody(envDepthViewInverse, envDepthViewMat);

  const XrFovf& fov = view.fov;
  float envDepthTan[4] = { std::tan(fov.angleLeft), std::tan(fov.angleRight),
    std::tan(fov.angleDown), std::tan(fov.angleUp) };

  // ------------------------------------------------------------------
  // Save GL state.
  // ------------------------------------------------------------------
  GLboolean colorMask[4];
  glGetBooleanv(GL_COLOR_WRITEMASK, colorMask);
  GLboolean depthMask;
  glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
  GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
  GLint depthFunc;
  glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);
  GLboolean blendEnabled = glIsEnabled(GL_BLEND);
  GLint blendSrcRGB, blendDstRGB, blendSrcAlpha, blendDstAlpha;
  glGetIntegerv(GL_BLEND_SRC_RGB, &blendSrcRGB);
  glGetIntegerv(GL_BLEND_DST_RGB, &blendDstRGB);
  glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendSrcAlpha);
  glGetIntegerv(GL_BLEND_DST_ALPHA, &blendDstAlpha);
  GLint prevProgram;
  glGetIntegerv(GL_CURRENT_PROGRAM, &prevProgram);
  GLint prevActiveTexUnit;
  glGetIntegerv(GL_ACTIVE_TEXTURE, &prevActiveTexUnit);
  GLint prevTexBinding;
  glActiveTexture(GL_TEXTURE0);
  glGetIntegerv(GL_TEXTURE_BINDING_2D_ARRAY, &prevTexBinding);
  GLint prevVAO;
  glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &prevVAO);

  // ------------------------------------------------------------------
  // Configure GL.
  //   - GL_LEQUAL depth test: passes when realWorldDepth <= sceneDepth
  //                           (virtual geometry is BEHIND the real world)
  //   - Depth mask false: do not overwrite the scene's depth values
  //   - Color mask alpha-only: only the alpha channel is modified
  //   - Multiplicative alpha blend: result.a = dst.a * occludedOpacity
  //     (empty pixels with dst.a=0 are unaffected; see shader comment)
  // ------------------------------------------------------------------
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_TRUE); // alpha only
  glEnable(GL_DEPTH_TEST);
  // GL_LEQUAL (rather than GL_LESS) handles the case where the real-world
  // surface sits at exactly the same NDC depth as the virtual geometry due
  // to the 1e-4 bias applied in the shader.
  glDepthFunc(GL_LEQUAL);
  glDepthMask(GL_FALSE);
  // Multiplicative alpha blend: result.a = dst.a * (1 - src.a).
  // With the shader outputting src.a = (1 - occludedOpacity) this gives
  //   result.a = dst.a * occludedOpacity
  // so only pixels that actually have rendered geometry (dst.a > 0) are
  // affected; empty/background pixels remain fully transparent.
  glEnable(GL_BLEND);
  glBlendFuncSeparate(GL_ZERO, GL_ONE, GL_ZERO, GL_ONE_MINUS_SRC_ALPHA);

  // ------------------------------------------------------------------
  // Draw.
  // ------------------------------------------------------------------
  glUseProgram(static_cast<GLuint>(this->OcclusionPostPassProgram));

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D_ARRAY, static_cast<GLuint>(envDepthGLTexture));
  glUniform1i(glGetUniformLocation(this->OcclusionPostPassProgram, "envDepthTex"), 0);

  // The OpenXR runtime typically creates the env-depth texture with GL_NEAREST
  // filtering.  Force GL_LINEAR so depth values interpolate bi-linearly across
  // texel boundaries, producing a smoothly-varying depth threshold at real-world
  // object edges rather than a staircase-aliased boundary.
  GLint origEnvDepthMinFilter, origEnvDepthMagFilter;
  glGetTexParameteriv(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, &origEnvDepthMinFilter);
  glGetTexParameteriv(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, &origEnvDepthMagFilter);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  auto loc = [&](const char* n) {
    return glGetUniformLocation(this->OcclusionPostPassProgram, n);
  };
  glUniformMatrix4fv(loc("vrProjInverse"),       1, GL_FALSE, vrProjInverse);
  glUniformMatrix4fv(loc("physicalToClip"),      1, GL_FALSE, physicalToClip);
  glUniformMatrix4fv(loc("vrViewInverse"),       1, GL_FALSE, vrViewInverse);
  glUniformMatrix4fv(loc("envDepthViewInverse"),  1, GL_FALSE, envDepthViewInverse);
  glUniformMatrix4fv(loc("envDepthView"),         1, GL_FALSE, envDepthViewMat);
  glUniform4fv(      loc("envDepthTan"),           1, envDepthTan);
  glUniform1i(       loc("eye"),                  static_cast<GLint>(eye));
  glUniform1f(       loc("occludedOpacity"),      occludedOpacity);
  glUniform1f(       loc("sigmaTexels"),          sigmaTexels);

  glBindVertexArray(static_cast<GLuint>(this->VAO));
  glDrawArrays(GL_TRIANGLES, 0, 3);

  // ------------------------------------------------------------------
  // Restore GL state.
  // ------------------------------------------------------------------
  glBindVertexArray(static_cast<GLuint>(prevVAO));
  glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
  glDepthMask(depthMask);
  glDepthFunc(static_cast<GLenum>(depthFunc));
  if (!depthTestEnabled)
  {
    glDisable(GL_DEPTH_TEST);
  }
  glBlendFuncSeparate(static_cast<GLenum>(blendSrcRGB), static_cast<GLenum>(blendDstRGB),
    static_cast<GLenum>(blendSrcAlpha), static_cast<GLenum>(blendDstAlpha));
  if (!blendEnabled)
  {
    glDisable(GL_BLEND);
  }
  glUseProgram(static_cast<GLuint>(prevProgram));
  // Restore env-depth texture filter settings before unbinding.
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, origEnvDepthMinFilter);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, origEnvDepthMagFilter);
  glBindTexture(GL_TEXTURE_2D_ARRAY, static_cast<GLuint>(prevTexBinding));
  glActiveTexture(static_cast<GLenum>(prevActiveTexUnit));
}

VTK_ABI_NAMESPACE_END
