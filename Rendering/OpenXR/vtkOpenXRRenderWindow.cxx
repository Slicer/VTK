// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-FileCopyrightText: Copyright (c) 2015, Valve Corporation
// SPDX-License-Identifier: BSD-3-Clause
#include "vtkOpenXRRenderWindow.h"

#include "vtkGlobFileNames.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLState.h"
#include "vtkOpenXRDefinitions.h"
#include "vtkOpenXRManager.h"
#include "vtkOpenXRModel.h"
#include "vtkOpenXRRenderWindowInteractor.h"
#include "vtkOpenXRRenderer.h"
#include "vtkOpenXRSceneObserver.h"
#include "vtkOpenXRUtilities.h"
#include "vtkRendererCollection.h"
#include "vtkResourceFileLocator.h"
#include "vtkVRCamera.h"
#include "vtkVersion.h"

#include "vtk_jsoncpp.h"
#include <vtksys/FStream.hxx>
#include <vtksys/SystemTools.hxx>

#include <map> // map

#ifdef XR_META_environment_depth
#include "vtkOpenXREnvironmentDepthOcclusionPrePass.h"
#endif

// include what we need for the helper window
#if defined(_WIN32)
#include "vtkWin32OpenGLRenderWindow.h"
#endif

#if defined(VTK_USE_X)
#include "vtkXOpenGLRenderWindow.h"
#endif

#if !defined(_WIN32) || defined(__CYGWIN__)
#define stricmp strcasecmp
#endif

VTK_ABI_NAMESPACE_BEGIN

class vtkOpenXRRenderWindow::vtkInternals
{
public:
  vtkInternals() = default;

  //----------------------------------------------------------------------------
  // OpenXR does not yet have a way to load and render controller
  // models. This workaround maps known/tested interaction profiles
  // to local assets.
  void LoadRenderModelMapping()
  {
    std::string modelsFile = "openxr_controllermodels.json";

    // Look for where the function "GetVTKVersion." lives.
    auto vtklib = vtkGetLibraryPathForSymbol(GetVTKVersion);

    if (vtklib.empty())
    {
      vtkWarningWithObjectMacro(nullptr, << "Unable to locate runtime library location.");
      return;
    }

    // Search for the controller models json file in the parent of the
    // directory containing the library/executable.
    const std::string libPath = vtksys::SystemTools::GetFilenamePath(vtklib);
    const std::string rootSearchDir = vtksys::SystemTools::GetParentDirectory(libPath);
    vtkNew<vtkGlobFileNames> fileGlobber;
    fileGlobber->SetDirectory(rootSearchDir.c_str());
    fileGlobber->SetRecurse(true);
    fileGlobber->AddFileNames(modelsFile.c_str());
    if (!this->ModelsManifestDirectory.empty())
    {
      std::string fullpath =
        vtksys::SystemTools::JoinPath({ this->ModelsManifestDirectory, modelsFile });
      fileGlobber->AddFileNames(fullpath.c_str());
    }
    if (fileGlobber->GetNumberOfFileNames() <= 0)
    {
      vtkWarningWithObjectMacro(
        nullptr, << "Unable to locate model mapping file in: " << rootSearchDir);
      return;
    }

    const char* match = fileGlobber->GetNthFileName(0);
    const std::string modelFile(&match[0]);

    // Get the directory containing the controller models json file, since
    // we found it.
    std::string modelsDirectory = vtksys::SystemTools::GetFilenamePath(modelFile);

    // Open the profile->asset mapping file
    vtksys::ifstream file;
    file.open(modelFile.c_str());
    if (!file.is_open())
    {
      vtkWarningWithObjectMacro(nullptr, << "Unable to open model mapping file : " << modelFile);
      return;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string formattedErrors;
    if (!Json::parseFromStream(builder, file, &root, &formattedErrors))
    {
      // Report failures and their locations in the document
      vtkWarningWithObjectMacro(nullptr, << "Failed to parse file with errors :" << endl
                                         << formattedErrors);
      return;
    }

    uint32_t leftHand = static_cast<uint32_t>(vtk::detail::vtkOpenXRManager::ControllerIndex::Left);
    uint32_t rightHand =
      static_cast<uint32_t>(vtk::detail::vtkOpenXRManager::ControllerIndex::Right);

    for (Json::Value::ArrayIndex i = 0; i < root.size(); ++i)
    {
      const Json::Value& nextMapping = root[i];
      std::string profileName = nextMapping["interaction_profile"].asString();
      const Json::Value& assetPaths = nextMapping["asset_paths"];

      std::string leftControllerPath = assetPaths["left_controller"].asString();
      std::string leftControllerPathFull =
        vtksys::SystemTools::CollapseFullPath(modelsDirectory + "/" + leftControllerPath);
      this->SetModelAsset(profileName, leftHand, leftControllerPathFull);

      std::string rightControllerPath = assetPaths["right_controller"].asString();
      std::string rightControllerPathFull =
        vtksys::SystemTools::CollapseFullPath(modelsDirectory + "/" + rightControllerPath);
      this->SetModelAsset(profileName, rightHand, rightControllerPathFull);
    }
  }

  //----------------------------------------------------------------------------
  // Set the model asset path for given interaction profile and hand
  void SetModelAsset(const std::string& profile, uint32_t hand, const std::string& assetPath)
  {
    this->ProfileToModelMapping[profile][hand] = assetPath;
  }

  //----------------------------------------------------------------------------
  // Get the model asset path for given interaction profile and hand
  std::string& GetModelAsset(const std::string& profile, uint32_t hand)
  {
    return this->ProfileToModelMapping[profile][hand];
  }

  std::map<uint32_t, std::string> CurrentInteractionProfiles;
  std::map<std::string, std::map<uint32_t, std::string>> ProfileToModelMapping;
  std::string ModelsManifestDirectory;
  vtkSmartPointer<vtkOpenXRSceneObserver> SceneObserver;
};

vtkStandardNewMacro(vtkOpenXRRenderWindow);

//------------------------------------------------------------------------------
vtkOpenXRRenderWindow::vtkOpenXRRenderWindow()
  : Internal(new vtkOpenXRRenderWindow::vtkInternals())
{
  this->StereoCapableWindow = 1;
  this->StereoRender = 1;
  this->UseOffScreenBuffers = true;
  this->Size[0] = 640;
  this->Size[1] = 720;
  this->Position[0] = 100;
  this->Position[1] = 100;
}

//------------------------------------------------------------------------------
vtkOpenXRRenderWindow::~vtkOpenXRRenderWindow()
{
  this->Finalize();

  vtkRenderer* ren;
  vtkCollectionSimpleIterator rit;
  this->Renderers->InitTraversal(rit);
  while ((ren = this->Renderers->GetNextRenderer(rit)))
  {
    ren->SetRenderWindow(nullptr);
  }
}

//------------------------------------------------------------------------------
// Create an interactor that will work with this renderer.
vtkRenderWindowInteractor* vtkOpenXRRenderWindow::MakeRenderWindowInteractor()
{
  this->Interactor = vtkOpenXRRenderWindowInteractor::New();
  this->Interactor->SetRenderWindow(this);
  return this->Interactor;
}

//------------------------------------------------------------------------------
bool vtkOpenXRRenderWindow::GetSizeFromAPI()
{
  auto& xrManager = vtk::detail::vtkOpenXRManager::GetInstance();

  std::tie(this->Size[0], this->Size[1]) = xrManager.GetRecommendedImageRectSize();

  return true;
}

//------------------------------------------------------------------------------
// Add a renderer to the list of renderers.
void vtkOpenXRRenderWindow::AddRenderer(vtkRenderer* ren)
{
  if (ren && !vtkOpenXRRenderer::SafeDownCast(ren))
  {
    vtkErrorMacro("vtkOpenXRRenderWindow::AddRenderer: Failed to add renderer of type "
      << ren->GetClassName() << ": A vtkOpenXRRenderer is expected");
    return;
  }
  this->Superclass::AddRenderer(ren);
}

//------------------------------------------------------------------------------
// Initialize the rendering window.
void vtkOpenXRRenderWindow::Initialize()
{
  if (this->VRInitialized)
  {
    return;
  }

  if (!this->HelperWindow)
  {
    vtkErrorMacro(<< "HelperWindow is not set");
    return;
  }

  // No need to set size of helper window as we own the window
  this->HelperWindow->SetDisplayId(this->GetGenericDisplayId());
  this->HelperWindow->SetShowWindow(false);
  this->HelperWindow->Initialize();

  this->MakeCurrent();
  this->OpenGLInit();

  auto& xrManager = vtk::detail::vtkOpenXRManager::GetInstance();
  // Always reset the preferred blend mode first so that a previous session's
  // setting does not carry over into a fresh session.
  xrManager.SetPreferredEnvironmentBlendMode(XR_ENVIRONMENT_BLEND_MODE_MAX_ENUM);
  if (this->UsePassthrough)
  {
    xrManager.SetPreferredEnvironmentBlendMode(XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND);
  }
  if (!xrManager.Initialize(this))
  {
    // Set to false because the above init of the HelperWindow sets it to true
    vtkErrorMacro(<< "Failed to initialize OpenXRManager");
    return;
  }

  // Initialize XR_FB_passthrough objects if passthrough was requested and the
  // extension is available.  xrPassthroughStartFB will be called from
  // BeginSession() once the session transitions to the READY state.
  if (this->UsePassthrough && xrManager.IsFBPassthroughSupported())
  {
    if (!xrManager.StartFBPassthrough())
    {
      vtkWarningMacro("Failed to initialize XR_FB_passthrough objects. "
                      "Passthrough will not be available.");
    }
  }

  if (this->EnableSceneUnderstanding && xrManager.IsSceneUnderstandingSupported())
  {
    this->Internal->SceneObserver = vtkSmartPointer<vtkOpenXRSceneObserver>::New();
    if (!this->Internal->SceneObserver->Initialize())
    {
      vtkWarningMacro("Failed to initialize scene observer");
      this->Internal->SceneObserver = nullptr;
    }
  }

  // Create one framebuffer per view
  this->CreateFramebuffers();

  std::tie(this->Size[0], this->Size[1]) = xrManager.GetRecommendedImageRectSize();

  vtkDebugMacro(<< "Size : " << this->Size[0] << ", " << this->Size[1]);

  std::string strWindowTitle = "VTK - " + xrManager.GetOpenXRPropertiesAsString();
  this->SetWindowName(strWindowTitle.c_str());

  // Load the interaction-profile-to-asset mapping
  this->Internal->LoadRenderModelMapping();

  this->VRInitialized = true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRRenderWindow::IsPassthroughActive() const
{
  auto& xrManager = vtk::detail::vtkOpenXRManager::GetInstance();
  // Passthrough is active either via the standard ALPHA_BLEND environment blend
  // mode, or via the Meta-specific XR_FB_passthrough composition layer.
  return xrManager.GetEnvironmentBlendMode() == XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND ||
    xrManager.IsFBPassthroughActive();
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::Finalize()
{
  if (!this->VRInitialized)
  {
    return;
  }

  // Release VTK-owned OpenGL resources while the GL context is still valid.
  // This must happen before HelperWindow->Finalize() destroys the context.
  this->MakeCurrent();
  this->ReleaseGraphicsResources(this);
#ifdef XR_META_environment_depth
  // Destroy the occlusion pre-pass GL programs/VAOs while the context is current.
  this->EnvDepthPrePass.reset();
#endif

  // Destroy the GL context before XR teardown. The Meta OpenXR runtime's
  // XR_FB_passthrough layer deadlocks during xrDestroyInstance when a GL
  // context is still current. Destroying the context first causes the
  // runtime to skip its internal GL cleanup (the driver reclaims those
  // resources anyway) and avoids the hang.
  if (this->HelperWindow && this->HelperWindow->GetGenericContext())
  {
    this->HelperWindow->Finalize();
  }

  vtk::detail::vtkOpenXRManager::GetInstance().Finalize();

  this->VRInitialized = false;
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::Render()
{
  auto& xrManager = vtk::detail::vtkOpenXRManager::GetInstance();

  if (this->Internal->SceneObserver)
  {
    this->Internal->SceneObserver->UpdateSceneData();
  }

  this->UpdateHMDMatrixPose();

  if (xrManager.GetShouldRenderCurrentFrame())
  {
    // Start rendering
    this->Superclass::Render();
  }
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::UpdateHMDMatrixPose()
{
  auto handle =
    this->GetDeviceHandleForOpenXRHandle(vtk::detail::vtkOpenXRManager::ControllerIndex::Head);
  auto device =
    this->GetDeviceForOpenXRHandle(vtk::detail::vtkOpenXRManager::ControllerIndex::Head);
  this->AddDeviceHandle(handle, device);

  // use left eye as stand in for HMD right now
  // todo add event for head pose

  const XrPosef* xrPose = vtk::detail::vtkOpenXRManager::GetInstance().GetViewPose(LEFT_EYE);
  if (xrPose == nullptr)
  {
    vtkErrorMacro(<< "No pose for left eye");
    return;
  }

  // Convert a XrPosef to a vtk view matrix
  vtkMatrix4x4* hmdToPhysicalMatrix = this->GetDeviceToPhysicalMatrixForDeviceHandle(handle);
  vtkOpenXRUtilities::SetMatrixFromXrPose(hmdToPhysicalMatrix, *xrPose);

  // update the camera values based on the pose
  vtkNew<vtkMatrix4x4> d2wMat;
  this->GetDeviceToWorldMatrixForDeviceHandle(handle, d2wMat);

  vtkRenderer* ren;
  vtkCollectionSimpleIterator rit;
  this->Renderers->InitTraversal(rit);
  while ((ren = this->Renderers->GetNextRenderer(rit)))
  {
    vtkVRCamera* cam = vtkVRCamera::SafeDownCast(ren->GetActiveCamera());
    if (cam && cam->GetTrackHMD())
    {
      cam->SetCameraFromDeviceToWorldMatrix(d2wMat, this->GetPhysicalScale());
      if (ren->GetLightFollowCamera())
      {
        ren->UpdateLightsGeometryToFollowCamera();
      }
    }
  }
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::StereoUpdate()
{
  this->Superclass::StereoUpdate();

#ifdef XR_META_environment_depth
  auto& xrManager = vtk::detail::vtkOpenXRManager::GetInstance();
  if (xrManager.IsEnvironmentDepthActive())
  {
    // Ensure the post-pass object exists — it handles occlusion post-pass and
    // debug visualisation. The post-pass runs AFTER each eye's scene render in
    // StereoMidpoint / StereoRenderComplete, so scene depths are already present
    // in the depth buffer.  This avoids the timing issue where a pre-pass written
    // here would be erased by VTK's renderer Clear() before geometry renders.
    if (!this->EnvDepthPrePass)
    {
      this->EnvDepthPrePass = std::make_unique<vtkOpenXREnvironmentDepthOcclusionPrePass>();
    }
    // OccludedOpacity < 1.0: ApplyOcclusionPostPass runs after each eye render.
    // OccludedOpacity == 1.0: depth composition bypassed entirely.
  }
#endif
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::StereoMidpoint()
{
  this->GetState()->vtkglDisable(GL_MULTISAMPLE);

#ifdef XR_META_environment_depth
  {
    auto& xrManager = vtk::detail::vtkOpenXRManager::GetInstance();
    if (xrManager.IsEnvironmentDepthActive() && this->EnvDepthPrePass)
    {
      vtkCamera* cam = this->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
      // Debug visualisation overlay (shown only when explicitly enabled).
      if (this->ShowEnvDepthDebugVisualization)
      {
        this->EnvDepthPrePass->DebugVisualize(LEFT_EYE, cam, xrManager.GetEnvDepthTextureId(),
          xrManager.GetEnvDepthViews(), static_cast<float>(this->GetPhysicalScale()));
      }
      // Post-pass: alpha-based occlusion uses the scene's depth buffer,
      // so it runs AFTER the scene render and BEFORE blitting the eye.
      // OccludedOpacity=0.0 fully hides occluded pixels (alpha → 0);
      // OccludedOpacity in (0,1) partially hides them.
      if (this->OccludedOpacity < 1.0f)
      {
        this->EnvDepthPrePass->ApplyOcclusionPostPass(LEFT_EYE, cam,
          xrManager.GetEnvDepthTextureId(), xrManager.GetEnvDepthViews(), this->OccludedOpacity,
          static_cast<float>(this->GetPhysicalScale()));
      }
    }
  }
#endif

  if (this->SwapBuffers)
  {
    this->RenderOneEye(LEFT_EYE);
  }
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::StereoRenderComplete()
{
  this->GetState()->vtkglDisable(GL_MULTISAMPLE);

#ifdef XR_META_environment_depth
  {
    auto& xrManager = vtk::detail::vtkOpenXRManager::GetInstance();
    if (xrManager.IsEnvironmentDepthActive() && this->EnvDepthPrePass)
    {
      vtkCamera* cam = this->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
      // Debug visualisation overlay.
      if (this->ShowEnvDepthDebugVisualization)
      {
        this->EnvDepthPrePass->DebugVisualize(RIGHT_EYE, cam, xrManager.GetEnvDepthTextureId(),
          xrManager.GetEnvDepthViews(), static_cast<float>(this->GetPhysicalScale()));
      }
      // Post-pass for the right eye.
      if (this->OccludedOpacity < 1.0f)
      {
        this->EnvDepthPrePass->ApplyOcclusionPostPass(RIGHT_EYE, cam,
          xrManager.GetEnvDepthTextureId(), xrManager.GetEnvDepthViews(), this->OccludedOpacity,
          static_cast<float>(this->GetPhysicalScale()));
      }
    }
  }
#endif

  if (this->SwapBuffers)
  {
    this->RenderOneEye(RIGHT_EYE);
  }
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::RenderOneEye(uint32_t eye)
{
  auto& xrManager = vtk::detail::vtkOpenXRManager::GetInstance();

  FramebufferDesc& eyeFramebufferDesc = this->FramebufferDescs[eye];

  if (!xrManager.PrepareRendering(
        this, &eyeFramebufferDesc.ResolveColorTextureId, &eyeFramebufferDesc.ResolveDepthTextureId))
  {
    return;
  }

  this->RenderModels();

  // When binding texture, the color texture id stored in the
  // framebufferDesc must be set
  this->BindTextureToFramebuffer(eyeFramebufferDesc);

  // For this eye, the rendering resources of OpenXRUtilities and the texture ids are set
  // we can render
  this->RenderFramebuffer(eyeFramebufferDesc);

  // Release this swapchain image
  xrManager.ReleaseSwapchainImage(eye);
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::RenderModels()
{
  vtkOpenGLState* ostate = this->GetState();
  ostate->vtkglEnable(GL_DEPTH_TEST);

  for (uint32_t hand : { vtk::detail::vtkOpenXRManager::ControllerIndex::Left,
         vtk::detail::vtkOpenXRManager::ControllerIndex::Right })
  {
    // Defer model loading until we have an interaction profile
    const std::string& currentProfile = this->GetCurrentInteractionProfile(hand);
    if (currentProfile.empty())
    {
      vtkDebugMacro(<< "Defer loading controller model for hand: " << hand);
      continue;
    }

    // do we not have a model loaded yet? try loading one
    auto handle = this->GetDeviceHandleForOpenXRHandle(hand);
    auto device = this->GetDeviceForOpenXRHandle(hand);
    this->AddDeviceHandle(handle, device);
    auto* pRenderModel = this->GetModelForDeviceHandle(handle);
    if (!pRenderModel)
    {
      vtkNew<vtkOpenXRModel> newModel;
      auto search = this->Internal->ProfileToModelMapping.find(currentProfile);
      if (search != this->Internal->ProfileToModelMapping.end())
      {
        auto modelMap = search->second;
        auto model = modelMap.find(hand);
        if (model != modelMap.end())
        {
          newModel->SetAssetPath(model->second);
        }
      }
      this->SetModelForDeviceHandle(handle, newModel);
      pRenderModel = newModel;
    }

    // if we have a model and it is visible
    if (pRenderModel && pRenderModel->GetVisibility())
    {
      vtkMatrix4x4* tdPose = this->GetDeviceToPhysicalMatrixForDeviceHandle(handle);
      pRenderModel->Render(this, tdPose);
    }
  }
}

//------------------------------------------------------------------------------
bool vtkOpenXRRenderWindow::CreateFramebuffers(uint32_t vtkNotUsed(viewCount))
{
  // With OpenXR, textures are created by the runtime because the compositor / runtime
  // knows better how to allocate a texture/buffer that will perform well
  // So we call glFrameBufferTexture2D at each frame with the texture provided by
  // the runtime
  // That's why we only generate framebuffers here
  auto& xrManager = vtk::detail::vtkOpenXRManager::GetInstance();
  uint32_t viewCount = xrManager.GetViewCount();
  this->FramebufferDescs.resize(viewCount);
  for (size_t i = 0; i < viewCount; ++i)
  {
    glGenFramebuffers(1, &this->FramebufferDescs[i].ResolveFramebufferId);
  }

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRRenderWindow::BindTextureToFramebuffer(FramebufferDesc& framebufferDesc)
{
  this->GetState()->PushFramebufferBindings();
  this->GetState()->vtkglBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.ResolveFramebufferId);

  glFramebufferTexture2D(
    GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferDesc.ResolveColorTextureId, 0);

  if (vtk::detail::vtkOpenXRManager::GetInstance().IsDepthExtensionSupported())
  {
    glFramebufferTexture2D(
      GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, framebufferDesc.ResolveDepthTextureId, 0);
  }

  // check FBO status
  GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE)
  {
    vtkErrorMacro(<< "Framebuffer binding is not complete");
    return false;
  }

  this->GetState()->PopFramebufferBindings();

  return true;
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::RenderFramebuffer(FramebufferDesc& framebufferDesc)
{
  // Blit the render frame buffer into the draw frame buffer
  this->GetState()->PushDrawFramebufferBinding();

  // We will read from actual read buffer and draw in our framebuffer
  this->GetState()->vtkglBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebufferDesc.ResolveFramebufferId);

  glBlitFramebuffer(0, 0, this->Size[0], this->Size[1], 0, 0, this->Size[0], this->Size[1],
    GL_COLOR_BUFFER_BIT, GL_LINEAR);

  if (vtk::detail::vtkOpenXRManager::GetInstance().IsDepthExtensionSupported())
  {
    glBlitFramebuffer(0, 0, this->Size[0], this->Size[1], 0, 0, this->Size[0], this->Size[1],
      GL_DEPTH_BUFFER_BIT, GL_NEAREST);
  }

  this->GetState()->PopDrawFramebufferBinding();
}

//------------------------------------------------------------------------------
uint32_t vtkOpenXRRenderWindow::GetDeviceHandleForOpenXRHandle(uint32_t index)
{
  return index;
}

vtkEventDataDevice vtkOpenXRRenderWindow::GetDeviceForOpenXRHandle(uint32_t ohandle)
{
  if (ohandle == vtk::detail::vtkOpenXRManager::ControllerIndex::Left)
  {
    return vtkEventDataDevice::LeftController;
  }
  if (ohandle == vtk::detail::vtkOpenXRManager::ControllerIndex::Right)
  {
    return vtkEventDataDevice::RightController;
  }
  if (ohandle == vtk::detail::vtkOpenXRManager::ControllerIndex::Head)
  {
    return vtkEventDataDevice::HeadMountedDisplay;
  }

  return vtkEventDataDevice::Unknown;
}

//------------------------------------------------------------------------------
std::string& vtkOpenXRRenderWindow::GetCurrentInteractionProfile(uint32_t hand)
{
  return this->Internal->CurrentInteractionProfiles[hand];
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::SetCurrentInteractionProfile(uint32_t hand, const std::string& profile)
{
  std::string currentValue = this->Internal->CurrentInteractionProfiles[hand];
  if (profile != currentValue)
  {
    this->Internal->CurrentInteractionProfiles[hand] = profile;
  }
}

//------------------------------------------------------------------------------
std::string& vtkOpenXRRenderWindow::GetModelsManifestDirectory()
{
  return this->Internal->ModelsManifestDirectory;
}

//------------------------------------------------------------------------------
void vtkOpenXRRenderWindow::SetModelsManifestDirectory(const std::string& path)
{
  this->Internal->ModelsManifestDirectory = path;
}

//------------------------------------------------------------------------------
vtkOpenXRSceneObserver* vtkOpenXRRenderWindow::GetSceneObserver()
{
  return this->Internal->SceneObserver;
}

//------------------------------------------------------------------------------
vtkOpenXRRenderWindow::InstanceVersion vtkOpenXRRenderWindow::QueryInstanceVersion(
  vtkOpenXRManagerConnection* cs)
{
  if (vtk::detail::vtkOpenXRManager::GetInstance().GetXrRuntimeInstance() == XR_NULL_HANDLE)
  {
    vtkWarningWithObjectMacro(nullptr,
      "Cannot call QueryInstanceVersion after a vtkOpenXRRenderWindow has been initialized.");
    return {};
  }

  if (!cs->Initialize())
  {
    vtkWarningWithObjectMacro(nullptr, "Failed to initialize connection strategy.");
    return {};
  }

  std::vector<const char*> enabledExtensions; // enable cs extension, if any
  if (std::strlen(cs->GetExtensionName()) != 0)
  {
    enabledExtensions.emplace_back(cs->GetExtensionName());
  }

  // Create the instance with enabled extensions.
  XrInstanceCreateInfo createInfo{ XR_TYPE_INSTANCE_CREATE_INFO };
  createInfo.applicationInfo = XrApplicationInfo{
    "OpenXR with VTK",    // .applicationName
    1,                    // .applicationVersion
    "",                   // .engineName
    1,                    // .engineVersion
#ifdef XR_API_VERSION_1_0 // available with OpenXR 1.1.37 or later:
    XR_API_VERSION_1_0,   // .apiVersion
#else                     // for 1.1.36 and earlier:
    XR_MAKE_VERSION(1, 0, XR_VERSION_PATCH(XR_CURRENT_API_VERSION)), // .apiVersion
#endif
  };
  createInfo.enabledExtensionCount = static_cast<uint32_t>(enabledExtensions.size());
  createInfo.enabledExtensionNames = enabledExtensions.data();

  XrInstance instance;
  if (xrCreateInstance(&createInfo, &instance) != XR_SUCCESS)
  {
    vtkWarningWithObjectMacro(nullptr, "Failed to create instance for version query.");
    return {};
  }

  XrInstanceProperties properties{ XR_TYPE_INSTANCE_PROPERTIES };
  if (xrGetInstanceProperties(instance, &properties))
  {
    vtkWarningWithObjectMacro(nullptr, "Failed to get instance properties.");
    return {};
  }

  InstanceVersion output;
  output.Major = XR_VERSION_MAJOR(properties.runtimeVersion);
  output.Minor = XR_VERSION_MINOR(properties.runtimeVersion);
  output.Patch = XR_VERSION_PATCH(properties.runtimeVersion);

  if (!cs->EndInitialize())
  {
    vtkWarningWithObjectMacro(nullptr, "Failed to terminate connection strategy initialization.");
  }

  xrDestroyInstance(instance);

  return output;
}

VTK_ABI_NAMESPACE_END
