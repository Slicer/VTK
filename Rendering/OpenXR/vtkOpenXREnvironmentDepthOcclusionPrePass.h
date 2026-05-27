// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkOpenXREnvironmentDepthOcclusionPrePass
 * @brief   Depth pre-pass for XR_META_environment_depth real-world occlusion.
 *
 * Before each eye's scene render this class runs a fullscreen GLSL pass that
 * samples the real-world depth texture acquired via xrAcquireEnvironmentDepthImageMETA
 * and writes corresponding values to gl_FragDepth, with colour writes disabled.
 *
 * The subsequent normal VTK scene render uses the default GL_LESS depth test.
 * Virtual geometry behind a real-world surface therefore fails the depth test
 * and contributes no colour; those pixels remain alpha = 0, letting the
 * passthrough camera feed show through.
 *
 * This class is only compiled when XR_META_environment_depth is available in
 * the OpenXR SDK headers.
 */

#ifndef vtkOpenXREnvironmentDepthOcclusionPrePass_h
#define vtkOpenXREnvironmentDepthOcclusionPrePass_h

#include "vtkRenderingOpenXRModule.h" // For export macro

// XrEnvironmentDepthImageViewMETA and related types.
// This header is only included inside an #ifdef XR_META_environment_depth guard
// in vtkOpenXRRenderWindow.cxx, so XR_META_environment_depth is defined here.
#include "vtkOpenXRPlatform.h"

#include <cstdint> // uint32_t

VTK_ABI_NAMESPACE_BEGIN

class vtkCamera;

class VTKRENDERINGOPENXR_EXPORT vtkOpenXREnvironmentDepthOcclusionPrePass
{
public:
  vtkOpenXREnvironmentDepthOcclusionPrePass();
  ~vtkOpenXREnvironmentDepthOcclusionPrePass();

  /**
   * Apply the depth pre-pass for the given eye.
   *
   * Fills the currently-bound FBO's depth buffer with real-world depths so
   * that virtual geometry behind physical surfaces fails the subsequent
   * GL_LESS depth test.
   *
   * @param eye              0 = left eye, 1 = right eye.
   * @param camera           The VR eye camera (provides projection/view matrices).
   * @param envDepthGLTexture GL name of the environment depth GL_TEXTURE_2D_ARRAY.
   * @param views            Per-eye view poses and FOV from XrEnvironmentDepthImageMETA.
   */
  void Apply(uint32_t eye, vtkCamera* camera, uint32_t envDepthGLTexture,
    const XrEnvironmentDepthImageViewMETA views[2], float physicalScale = 1.0f);

  /**
   * Render a false-colour depth visualisation overlay for debugging.
   *
   * Uses the same reprojection math as Apply() but writes colour pixels
   * (alpha-blended on top of the scene) instead of depth values.
   * Pixels that fall outside the env-depth camera frustum are discarded.
   * Valid depth values are mapped: red = near (0 m), blue = far (5 m).
   *
   * Call this AFTER the scene render but BEFORE submitting the eye to the
   * compositor (i.e. before RenderOneEye / swapchain release).
   */
  void DebugVisualize(uint32_t eye, vtkCamera* camera, uint32_t envDepthGLTexture,
    const XrEnvironmentDepthImageViewMETA views[2]);

  /**
   * Post-pass for partial real-world occlusion.
   *
   * Run AFTER the scene render (and DebugVisualize if desired), BEFORE
   * submitting the eye to the compositor.
   *
   * The env-depth texture is Gaussian-blurred (in its own texel space) before
   * the depth comparison, smoothing away blockiness caused by the limited
   * resolution of the depth sensor.  The blurred depth values are then
   * compared against the scene's depth buffer: pixels where the real-world
   * surface is closer than the rendered virtual geometry have their alpha
   * multiplied by @p occludedOpacity:
   *
   *   - occludedOpacity = 0.0  → fully occluded  (alpha → 0, invisible)
   *   - occludedOpacity = 0.5  → half transparent (alpha → alpha * 0.5)
   *   - (1.0 skips this pass entirely — caller should not call it)
   *
   * The currently-bound framebuffer's depth buffer must contain the scene's
   * rendered depths (i.e. no depth pre-pass should be used when calling this).
   *
   * @param sigmaTexels  Gaussian sigma in texels of the env-depth texture.
   *                     Controls how much the depth image is blurred before
   *                     comparison. Values in [1, 5] are typical; default 3.
   */
  void ApplyOcclusionPostPass(uint32_t eye, vtkCamera* camera, uint32_t envDepthGLTexture,
    const XrEnvironmentDepthImageViewMETA views[2], float occludedOpacity,
    float physicalScale = 1.0f, float sigmaTexels = 3.0f);

private:
  void BuildProgram();
  void BuildDebugProgram();
  void BuildOcclusionPostPassProgram();

  uint32_t Program{ 0 };                   // GL program object (same width as GLuint)
  uint32_t DebugProgram{ 0 };              // GL program for debug visualisation
  uint32_t OcclusionPostPassProgram{ 0 };  // GL program for partial occlusion post-pass
  uint32_t VAO{ 0 };                       // Empty VAO for attribute-less fullscreen triangle
};

VTK_ABI_NAMESPACE_END

#endif // vtkOpenXREnvironmentDepthOcclusionPrePass_h
