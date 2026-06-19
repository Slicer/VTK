// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
#include "vtkOpenXRManager.h"

#include "vtkCamera.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkOpenXRManagerOpenGLGraphics.h"
#include "vtkOpenXRRenderWindow.h"
#include "vtkOpenXRSceneObserver.h"
#include "vtkOpenXRUtilities.h"
#include "vtkRendererCollection.h"
#include "vtkStringFormatter.h"
#include "vtkWindows.h" // Does nothing if we are not on windows

// XR_USE_GRAPHICS_API_OPENGL must be defined before vtkOpenXRPlatform.h to
// obtain XrSwapchainImageOpenGLKHR (needed for environment depth swapchain).
#define XR_USE_GRAPHICS_API_OPENGL
#include "vtkOpenXRPlatform.h"

#include <cstring>

#include <iostream>

#define VTK_CHECK_NULL_XRHANDLE(handle, msg)                                                       \
  if (handle == XR_NULL_HANDLE)                                                                    \
  {                                                                                                \
    vtkErrorWithObjectMacro(nullptr, << msg << " is a null handle.");                              \
    return false;                                                                                  \
  }

namespace vtk::detail
{
VTK_ABI_NAMESPACE_BEGIN

//------------------------------------------------------------------------------
vtkOpenXRManager::vtkOpenXRManager()
{
  // Use OpenGL as default backend
  this->GraphicsStrategy = vtkSmartPointer<vtkOpenXRManagerOpenGLGraphics>::New();

  // Use no-op connection strategy as default
  this->ConnectionStrategy = vtkSmartPointer<vtkOpenXRManagerConnection>::New();
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::Initialize(vtkOpenXRRenderWindow* xrWindow)
{
  vtkOpenGLRenderWindow* helperWindow = xrWindow->GetHelperWindow();

  if (!this->ConnectionStrategy->Initialize())
  {
    vtkWarningWithObjectMacro(nullptr, "Failed to initialize connection strategy.");
    return false;
  }

  if (!this->CreateInstance(xrWindow))
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed to CreateInstance");
    return false;
  }

  // Create the SubactionPaths (left / right hand and head)
  if (!this->CreateSubactionPaths())
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed to CreateSubactionPaths");
    return false;
  }

  if (!this->CreateSystem())
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed to CreateSystem");
    return false;
  }

  if (!this->GraphicsStrategy->CheckGraphicsRequirements(*this))
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed in CheckGraphicsRequirements");
    return false;
  }

  if (!this->GraphicsStrategy->CreateGraphicsBinding(helperWindow))
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed to CreateGraphicsBinding");
    return false;
  }

  // When using remoting, the connection must be established before creating the session
  if (!this->ConnectionStrategy->ConnectToRemote(*this))
  {
    vtkWarningWithObjectMacro(nullptr, "Failed to connect.");
    return false;
  }

  if (!this->CreateSession())
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed to CreateSession");
    return false;
  }

  // System properties use the following functions that must be called after
  // the connection has succeeded when using remoting:
  // xrEnumerateViewConfigurations, xrGetViewConfigurationProperties,
  // xrEnumerateEnvironmentBlendModes, xrGetSystemProperties.
  if (!this->CreateSystemProperties())
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed to CreateSystemProperties");
    return false;
  }

  if (!this->CreateReferenceSpace())
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed to CreateReferenceSpace");
    return false;
  }

  if (!this->CreateSwapchains())
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed to CreateSwapChains");
    return false;
  }

  if (!this->LoadControllerModels())
  {
    vtkWarningWithObjectMacro(nullptr, "Initialize failed to LoadController Models");
    return false;
  }

  if (!this->ConnectionStrategy->EndInitialize())
  {
    vtkWarningWithObjectMacro(nullptr, "Failed to terminate connection strategy initialization.");
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::Finalize()
{
  // Stop environment depth and FB passthrough before destroying the session/instance.
  this->StopEnvironmentDepth();
  this->StopFBPassthrough();
  this->DestroyActionSets();
  xrRequestExitSession(this->Session);
  xrEndSession(this->Session);
  xrDestroySession(this->Session);
  xrDestroyInstance(this->Instance);
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::StartFBPassthrough()
{
#ifdef XR_FB_passthrough
  if (!this->OptionalExtensions.FBPassthroughSupported)
  {
    vtkWarningWithObjectMacro(nullptr, "XR_FB_passthrough is not supported by this runtime.");
    return false;
  }

  xr::ExtensionDispatchTable ext;
  ext.PopulateDispatchTable(this->Instance);

  if (!ext.xrCreatePassthroughFB)
  {
    vtkWarningWithObjectMacro(
      nullptr, "xrCreatePassthroughFB function pointer could not be loaded.");
    return false;
  }

  // Create the passthrough object (requires session in IDLE or later state).
  XrPassthroughCreateInfoFB passthroughInfo{ XR_TYPE_PASSTHROUGH_CREATE_INFO_FB };
  passthroughInfo.flags = 0;
  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        ext.xrCreatePassthroughFB(this->Session, &passthroughInfo, &this->FBPassthrough),
        "Failed to create XR_FB_passthrough object"))
  {
    return false;
  }

  // Create a full-reconstruction passthrough layer (covers the full view).
  XrPassthroughLayerCreateInfoFB layerInfo{ XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB };
  layerInfo.passthrough = this->FBPassthrough;
  layerInfo.purpose = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB;
  layerInfo.flags = XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB;
  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        ext.xrCreatePassthroughLayerFB(this->Session, &layerInfo, &this->FBPassthroughLayer),
        "Failed to create XR_FB_passthrough layer"))
  {
    ext.xrDestroyPassthroughFB(this->FBPassthrough);
    this->FBPassthrough = XR_NULL_HANDLE;
    return false;
  }

  // Note: xrPassthroughStartFB is called from BeginSession() once the session
  // is running (XR_SESSION_STATE_READY or later).
  return true;
#else
  vtkWarningWithObjectMacro(
    nullptr, "XR_FB_passthrough is not available in this OpenXR SDK build.");
  return false;
#endif
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::StopFBPassthrough()
{
#ifdef XR_FB_passthrough
  // Clean up resources even if passthrough was never successfully started.
  if (this->FBPassthrough == XR_NULL_HANDLE && this->FBPassthroughLayer == XR_NULL_HANDLE)
  {
    this->FBPassthroughActive = false;
    return;
  }

  xr::ExtensionDispatchTable ext;
  ext.PopulateDispatchTable(this->Instance);

  if (this->FBPassthroughActive && ext.xrPassthroughPauseFB &&
    this->FBPassthrough != XR_NULL_HANDLE)
  {
    ext.xrPassthroughPauseFB(this->FBPassthrough);
  }
  if (ext.xrDestroyPassthroughLayerFB && this->FBPassthroughLayer != XR_NULL_HANDLE)
  {
    ext.xrDestroyPassthroughLayerFB(this->FBPassthroughLayer);
    this->FBPassthroughLayer = XR_NULL_HANDLE;
  }
  if (ext.xrDestroyPassthroughFB && this->FBPassthrough != XR_NULL_HANDLE)
  {
    ext.xrDestroyPassthroughFB(this->FBPassthrough);
    this->FBPassthrough = XR_NULL_HANDLE;
  }

  this->FBPassthroughActive = false;
#endif
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::StartEnvironmentDepth()
{
#ifdef XR_META_environment_depth
  if (!this->OptionalExtensions.EnvironmentDepthSupported)
  {
    vtkWarningWithObjectMacro(
      nullptr, "XR_META_environment_depth is not supported by this runtime.");
    return false;
  }

  xr::ExtensionDispatchTable ext;
  ext.PopulateDispatchTable(this->Instance);

  if (!ext.xrCreateEnvironmentDepthProviderMETA)
  {
    vtkWarningWithObjectMacro(
      nullptr, "xrCreateEnvironmentDepthProviderMETA function pointer could not be loaded.");
    return false;
  }

  // Create the depth provider.
  XrEnvironmentDepthProviderCreateInfoMETA providerInfo{
    XR_TYPE_ENVIRONMENT_DEPTH_PROVIDER_CREATE_INFO_META
  };
  providerInfo.createFlags = 0;
  if (!this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
        ext.xrCreateEnvironmentDepthProviderMETA(
          this->Session, &providerInfo, &this->EnvDepthProvider),
        "Failed to create XR_META_environment_depth provider"))
  {
    return false;
  }

  // Create the depth swapchain.
  XrEnvironmentDepthSwapchainCreateInfoMETA swapchainInfo{
    XR_TYPE_ENVIRONMENT_DEPTH_SWAPCHAIN_CREATE_INFO_META
  };
  swapchainInfo.createFlags = 0;
  if (!this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
        ext.xrCreateEnvironmentDepthSwapchainMETA(
          this->EnvDepthProvider, &swapchainInfo, &this->EnvDepthSwapchain),
        "Failed to create XR_META_environment_depth swapchain"))
  {
    ext.xrDestroyEnvironmentDepthProviderMETA(this->EnvDepthProvider);
    this->EnvDepthProvider = XR_NULL_HANDLE;
    return false;
  }

  // Enumerate swapchain images and store their GL texture IDs.
  uint32_t imageCount = 0;
  if (!this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
        ext.xrEnumerateEnvironmentDepthSwapchainImagesMETA(
          this->EnvDepthSwapchain, 0, &imageCount, nullptr),
        "Failed to get environment depth swapchain image count") ||
    imageCount == 0)
  {
    vtkWarningWithObjectMacro(nullptr, "Environment depth swapchain returned zero images.");
    ext.xrDestroyEnvironmentDepthSwapchainMETA(this->EnvDepthSwapchain);
    this->EnvDepthSwapchain = XR_NULL_HANDLE;
    ext.xrDestroyEnvironmentDepthProviderMETA(this->EnvDepthProvider);
    this->EnvDepthProvider = XR_NULL_HANDLE;
    return false;
  }

  std::vector<XrSwapchainImageOpenGLKHR> images(imageCount, { XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR });
  if (!this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
        ext.xrEnumerateEnvironmentDepthSwapchainImagesMETA(this->EnvDepthSwapchain, imageCount,
          &imageCount, reinterpret_cast<XrSwapchainImageBaseHeader*>(images.data())),
        "Failed to enumerate environment depth swapchain images"))
  {
    ext.xrDestroyEnvironmentDepthSwapchainMETA(this->EnvDepthSwapchain);
    this->EnvDepthSwapchain = XR_NULL_HANDLE;
    ext.xrDestroyEnvironmentDepthProviderMETA(this->EnvDepthProvider);
    this->EnvDepthProvider = XR_NULL_HANDLE;
    return false;
  }

  this->EnvDepthTextureIds.resize(imageCount);
  for (uint32_t i = 0; i < imageCount; ++i)
  {
    this->EnvDepthTextureIds[i] = images[i].image;
  }

  // Start the provider — this initiates depth capture.
  if (!this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
        ext.xrStartEnvironmentDepthProviderMETA(this->EnvDepthProvider),
        "Failed to start XR_META_environment_depth provider"))
  {
    ext.xrDestroyEnvironmentDepthSwapchainMETA(this->EnvDepthSwapchain);
    this->EnvDepthSwapchain = XR_NULL_HANDLE;
    ext.xrDestroyEnvironmentDepthProviderMETA(this->EnvDepthProvider);
    this->EnvDepthProvider = XR_NULL_HANDLE;
    return false;
  }

  this->EnvDepthActive = true;
  vtkDebugWithObjectMacro(
    nullptr, "Environment depth provider started (" << imageCount << " swapchain images).");
  return true;
#else
  return false;
#endif
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::StopEnvironmentDepth()
{
#ifdef XR_META_environment_depth
  if (!this->EnvDepthActive)
  {
    return;
  }

  xr::ExtensionDispatchTable ext;
  ext.PopulateDispatchTable(this->Instance);

  if (ext.xrStopEnvironmentDepthProviderMETA && this->EnvDepthProvider != XR_NULL_HANDLE)
  {
    ext.xrStopEnvironmentDepthProviderMETA(this->EnvDepthProvider);
  }
  if (ext.xrDestroyEnvironmentDepthSwapchainMETA && this->EnvDepthSwapchain != XR_NULL_HANDLE)
  {
    ext.xrDestroyEnvironmentDepthSwapchainMETA(this->EnvDepthSwapchain);
    this->EnvDepthSwapchain = XR_NULL_HANDLE;
  }
  if (ext.xrDestroyEnvironmentDepthProviderMETA && this->EnvDepthProvider != XR_NULL_HANDLE)
  {
    ext.xrDestroyEnvironmentDepthProviderMETA(this->EnvDepthProvider);
    this->EnvDepthProvider = XR_NULL_HANDLE;
  }

  this->EnvDepthTextureIds.clear();
  this->EnvDepthActive = false;
#endif
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::AcquireEnvironmentDepthImage()
{
#ifdef XR_META_environment_depth
  xr::ExtensionDispatchTable ext;
  ext.PopulateDispatchTable(this->Instance);

  if (!ext.xrAcquireEnvironmentDepthImageMETA)
  {
    return false;
  }

  XrEnvironmentDepthImageAcquireInfoMETA acquireInfo{
    XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_ACQUIRE_INFO_META
  };
  acquireInfo.space = this->ReferenceSpace;
  acquireInfo.displayTime = this->CurrentFrame.FrameState.predictedDisplayTime;

  XrEnvironmentDepthImageMETA depthImage{ XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_META };
  depthImage.views[0] = { XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_VIEW_META };
  depthImage.views[1] = { XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_VIEW_META };

  if (!this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
        ext.xrAcquireEnvironmentDepthImageMETA(this->EnvDepthProvider, &acquireInfo, &depthImage),
        "Failed to acquire environment depth image"))
  {
    return false;
  }

  if (depthImage.swapchainIndex >= this->EnvDepthTextureIds.size())
  {
    vtkWarningWithObjectMacro(nullptr,
      "Environment depth swapchain index " << depthImage.swapchainIndex << " is out of range ("
                                           << this->EnvDepthTextureIds.size() << " images).");
    return false;
  }

  this->EnvDepthTextureId = this->EnvDepthTextureIds[depthImage.swapchainIndex];
  this->EnvDepthViews[0] = depthImage.views[0];
  this->EnvDepthViews[1] = depthImage.views[1];
  this->EnvDepthNearZ = depthImage.nearZ;
  this->EnvDepthFarZ = depthImage.farZ;
  return true;
#else
  return false;
#endif
}

//------------------------------------------------------------------------------
std::tuple<uint32_t, uint32_t> vtkOpenXRManager::GetRecommendedImageRectSize()
{
  if (this->RenderResources->ConfigViews.empty())
  {
    return std::make_tuple(0, 0);
  }
  return std::make_tuple(this->RenderResources->ConfigViews[0].recommendedImageRectWidth,
    this->RenderResources->ConfigViews[0].recommendedImageRectHeight);
}

//------------------------------------------------------------------------------
uint32_t vtkOpenXRManager::GetRecommendedSampleCount()
{
  if (this->RenderResources->ConfigViews.empty())
  {
    return 0;
  }
  return this->RenderResources->ConfigViews[0].recommendedSwapchainSampleCount;
}

//------------------------------------------------------------------------------
std::string vtkOpenXRManager::GetOpenXRPropertiesAsString()
{
  XrInstanceProperties instanceProperties = {
    XR_TYPE_INSTANCE_PROPERTIES, // .type
    nullptr,                     // .next
  };
  if (!this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
        xrGetInstanceProperties(this->Instance, &instanceProperties),
        "Failed to get instance info"))
  {
    return "";
  }

  std::string properties = std::string(instanceProperties.runtimeName) + " " +
    vtk::to_string(XR_VERSION_MAJOR(instanceProperties.runtimeVersion)) + "." +
    vtk::to_string(XR_VERSION_MINOR(instanceProperties.runtimeVersion)) + "." +
    vtk::to_string(XR_VERSION_PATCH(instanceProperties.runtimeVersion));

  return properties;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::BeginSession()
{
  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::BeginSession, Session");

  XrSessionBeginInfo session_begin_info = {
    XR_TYPE_SESSION_BEGIN_INFO, // .type
    nullptr,                    // .next
    this->ViewType              // .primaryViewConfigurationType
  };
  if (!this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
        xrBeginSession(this->Session, &session_begin_info), "Failed to begin session!"))
  {
    return false;
  }

  vtkDebugWithObjectMacro(nullptr, "Session started.");

  this->SessionRunning = true;

#ifdef XR_FB_passthrough
  // Activate FB passthrough now that the session is running.
  // The XrPassthroughFB and XrPassthroughLayerFB handles were created by
  // StartFBPassthrough() while the session was still in IDLE state.
  if (this->FBPassthrough != XR_NULL_HANDLE && !this->FBPassthroughActive)
  {
    xr::ExtensionDispatchTable ext;
    ext.PopulateDispatchTable(this->Instance);
    if (ext.xrPassthroughStartFB)
    {
      if (this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
            ext.xrPassthroughStartFB(this->FBPassthrough), "Failed to start XR_FB_passthrough"))
      {
        this->FBPassthroughActive = true;
        vtkDebugWithObjectMacro(nullptr, "FB passthrough started.");
      }
    }
  }
#endif

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::WaitAndBeginFrame()
{
  // If a frame has already been started, skip xrWaitFrame/xrBeginFrame
  // they must only happen once per frame.
  if (this->FrameBegan)
  {
    return true;
  }

  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::WaitAndBeginFrame, Session");

  // Reset per-frame validity flags; FrameState/Views/ViewState are written
  // below or by LocateViews().
  this->CurrentFrame.ShouldRender = false;
  this->CurrentFrame.PoseValid = false;

  XrFrameWaitInfo frameWaitInfo{ XR_TYPE_FRAME_WAIT_INFO };
  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrWaitFrame(this->Session, &frameWaitInfo, &this->CurrentFrame.FrameState),
        "Failed to wait frame."))
  {
    return false;
  }

  XrFrameBeginInfo frameBeginInfo{ XR_TYPE_FRAME_BEGIN_INFO };
  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrBeginFrame(this->Session, &frameBeginInfo), "Failed to begin frame."))
  {
    return false;
  }

  this->CurrentFrame.ShouldRender = this->CurrentFrame.FrameState.shouldRender;

  this->FrameBegan = true;
  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::LocateViews()
{
  if (!this->CurrentFrame.ShouldRender)
  {
    // No need to locate views if the runtime has already indicated that we
    // shouldn't render this frame.
    return true;
  }

  XrViewLocateInfo viewLocateInfo{ XR_TYPE_VIEW_LOCATE_INFO };
  viewLocateInfo.viewConfigurationType = this->ViewType;
  viewLocateInfo.displayTime = this->CurrentFrame.FrameState.predictedDisplayTime;
  viewLocateInfo.space = this->ReferenceSpace;
  uint32_t viewCountOutput = 0;

  if (!this->XrCheckOutput(ErrorOutput,
        xrLocateViews(this->Session, &viewLocateInfo, &this->CurrentFrame.ViewState,
          (uint32_t)this->CurrentFrame.Views.size(), &viewCountOutput,
          this->CurrentFrame.Views.data()),
        "xrLocateViews failed"))
  {
    return false;
  }

  // Validate viewStateFlags. If invalid, fall back to the last known-good
  // pose so we still submit a coherent projection layer (avoids compositor
  // crashes / black frames). If we have never seen a valid pose, suppress
  // rendering for this frame entirely.
  const XrViewStateFlags requiredFlags =
    XR_VIEW_STATE_POSITION_VALID_BIT | XR_VIEW_STATE_ORIENTATION_VALID_BIT;
  const bool nowValid =
    (this->CurrentFrame.ViewState.viewStateFlags & requiredFlags) == requiredFlags;

  if (nowValid)
  {
    this->LastValidViewState = this->CurrentFrame.ViewState;
    this->LastValidViews = this->CurrentFrame.Views;
    this->CurrentFrame.PoseValid = true;
  }
  else if ((this->LastValidViewState.viewStateFlags & requiredFlags) == requiredFlags)
  {
    this->CurrentFrame.ViewState = this->LastValidViewState;
    this->CurrentFrame.Views = this->LastValidViews;
    this->CurrentFrame.PoseValid = true;
  }
  else
  {
    // No valid pose ever - skip submitting any projection layer this frame.
    this->CurrentFrame.PoseValid = false;
    this->CurrentFrame.ShouldRender = false;
  }

#ifdef XR_META_environment_depth
  if (this->OptionalExtensions.EnvironmentDepthSupported && !this->EnvDepthActive)
  {
    this->StartEnvironmentDepth();
  }
  if (this->EnvDepthActive)
  {
    this->AcquireEnvironmentDepthImage();
  }
#endif
  return true;
}

//------------------------------------------------------------------------------
// loads the controller models using an extension if it is present.
// todo needs to be tied into the models class and
// the gltf conversion completed right now it is here as an example
// to start from.
bool vtkOpenXRManager::LoadControllerModels()
{
  if (!this->OptionalExtensions.ControllerModelExtensionSupported)
  {
    return true;
  }

  // Controllers are not loaded when remoting to the hololens.
  // TODO: handle hand mesh tracking using XR_MSFT_hand_tracking_mesh extension.
  if (this->OptionalExtensions.RemotingSupported)
  {
    return true;
  }

  auto lPath = this->GetXrPath("/user/hand/left");

  xr::ExtensionDispatchTable extensions;
  // Define the pointer function of enabled extensions (see XrExtensions.h)
  extensions.PopulateDispatchTable(this->Instance);

  XrControllerModelKeyStateMSFT controllerModelKeyState;
  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    extensions.xrGetControllerModelKeyMSFT(this->Session, lPath, &controllerModelKeyState),
    "Failed to get controller model key!");

  // get the size
  uint32_t bufferCountOutput = 0;
  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    extensions.xrLoadControllerModelMSFT(
      this->Session, controllerModelKeyState.modelKey, 0, &bufferCountOutput, nullptr),
    "Failed to get controller model size!");

  // get the data
  uint32_t bufferCapacityInput = bufferCountOutput;
  uint8_t* buffer = new uint8_t[bufferCountOutput];
  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    extensions.xrLoadControllerModelMSFT(this->Session, controllerModelKeyState.modelKey,
      bufferCapacityInput, &bufferCountOutput, buffer),
    "Failed to get controller model!");
  delete[] buffer;

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::PrepareRendering(
  vtkOpenXRRenderWindow* win, void* colorTextureId, void* depthTextureId)
{
  // vtkOpenXRRenderWindow only supports a single renderer, so this is OK.
  vtkCamera* camera = win->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
  const std::uint32_t eye = camera->GetLeftEye() == 1 ? 0 : 1;

  const vtkOpenXRManager::Swapchain_t& colorSwapchain = this->RenderResources->ColorSwapchains[eye];
  const vtkOpenXRManager::Swapchain_t& depthSwapchain = this->RenderResources->DepthSwapchains[eye];

  // Use the full size of the allocated swapchain image (could render smaller some frames to hit
  // framerate)
  const XrRect2Di imageRect = { { 0, 0 },
    { (int32_t)colorSwapchain.Width, (int32_t)colorSwapchain.Height } };

  if (this->OptionalExtensions.DepthExtensionSupported)
  {
    if (colorSwapchain.Width != depthSwapchain.Width)
    {
      vtkErrorWithObjectMacro(nullptr, << "Color swapchain width (" << colorSwapchain.Width
                                       << ") differs from depth swapchain width ("
                                       << depthSwapchain.Width << ").");
      return false;
    }
    if (colorSwapchain.Height != depthSwapchain.Height)
    {
      vtkErrorWithObjectMacro(nullptr, << "Color swapchain height (" << colorSwapchain.Height
                                       << ") differs from depth swapchain height ("
                                       << depthSwapchain.Height << ").");
      return false;
    }
  }

  // Store the texture to render into it during the render method
  const uint32_t colorSwapchainImageIndex =
    this->WaitAndAcquireSwapchainImage(colorSwapchain.Swapchain);

  this->GraphicsStrategy->GetColorSwapchainImage(eye, colorSwapchainImageIndex, colorTextureId);

  this->RenderResources->ProjectionLayerViews[eye] = { XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW };
  this->RenderResources->ProjectionLayerViews[eye].pose = this->CurrentFrame.Views[eye].pose;
  this->RenderResources->ProjectionLayerViews[eye].fov = this->CurrentFrame.Views[eye].fov;
  this->RenderResources->ProjectionLayerViews[eye].subImage.swapchain = colorSwapchain.Swapchain;
  this->RenderResources->ProjectionLayerViews[eye].subImage.imageRect = imageRect;
  this->RenderResources->ProjectionLayerViews[eye].subImage.imageArrayIndex = 0;

  if (this->OptionalExtensions.DepthExtensionSupported)
  {
    const uint32_t depthSwapchainImageIndex =
      this->WaitAndAcquireSwapchainImage(depthSwapchain.Swapchain);

    this->GraphicsStrategy->GetDepthSwapchainImage(eye, depthSwapchainImageIndex, depthTextureId);

    std::array<double, 2> clippingPlanes;
    camera->GetClippingRange(clippingPlanes.data());

    double physscale = win->GetPhysicalScale();
    double znear = clippingPlanes[0] / physscale;
    double zfar = clippingPlanes[1] / physscale;

    this->RenderResources->DepthInfoViews[eye] = { XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR };
    this->RenderResources->DepthInfoViews[eye].minDepth = 0;
    this->RenderResources->DepthInfoViews[eye].maxDepth = 1;
    this->RenderResources->DepthInfoViews[eye].nearZ = znear;
    this->RenderResources->DepthInfoViews[eye].farZ = zfar;
    this->RenderResources->DepthInfoViews[eye].subImage.swapchain = depthSwapchain.Swapchain;
    this->RenderResources->DepthInfoViews[eye].subImage.imageRect = imageRect;
    this->RenderResources->DepthInfoViews[eye].subImage.imageArrayIndex = 0;

    // Chain depth info struct to the corresponding projection layer view's next pointer
    this->RenderResources->ProjectionLayerViews[eye].next =
      &this->RenderResources->DepthInfoViews[eye];
  }

  return true;
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::ReleaseSwapchainImage(uint32_t eye)
{
  XrSwapchainImageReleaseInfo releaseInfo{ XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };

  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrReleaseSwapchainImage(this->RenderResources->ColorSwapchains[eye].Swapchain, &releaseInfo),
    "Failed to release color swapchain image!");

  if (this->OptionalExtensions.DepthExtensionSupported)
  {
    this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
      xrReleaseSwapchainImage(this->RenderResources->DepthSwapchains[eye].Swapchain, &releaseInfo),
      "Failed to release depth swapchain image!");
  }
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::EndFrame()
{
  // Snapshot the render flag from CurrentFrame.
  const bool shouldRender = this->CurrentFrame.ShouldRender;

  XrCompositionLayerProjection layer{ XR_TYPE_COMPOSITION_LAYER_PROJECTION };

#ifdef XR_FB_passthrough
  XrCompositionLayerPassthroughFB passthroughCompositionLayer{
    XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB
  };
#endif

  std::vector<XrCompositionLayerBaseHeader*> layers;

#ifdef XR_FB_passthrough
  if (this->FBPassthroughActive && this->FBPassthroughLayer != XR_NULL_HANDLE)
  {
    passthroughCompositionLayer.flags = 0;
    passthroughCompositionLayer.space = XR_NULL_HANDLE;
    passthroughCompositionLayer.layerHandle = this->FBPassthroughLayer;

    layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&passthroughCompositionLayer));
  }
#endif

  // Projection layer
  if (shouldRender)
  {
    const uint32_t viewCount =
      static_cast<uint32_t>(this->RenderResources->ProjectionLayerViews.size());

    // Guard against mismatch (prevents xrEndFrame rejection / undefined compositor behavior)
    if (viewCount == 0 || viewCount != this->GetViewCount())
    {
      // Still end the frame to keep the frame loop consistent, but avoid
      // submitting an invalid projection layer.
      XrFrameEndInfo frameEndInfo{ XR_TYPE_FRAME_END_INFO };
      frameEndInfo.displayTime = this->CurrentFrame.FrameState.predictedDisplayTime;
      frameEndInfo.environmentBlendMode = this->EnvironmentBlendMode;
      frameEndInfo.layerCount = static_cast<uint32_t>(layers.size());
      frameEndInfo.layers = layers.empty() ? nullptr : layers.data();
      const bool ok = this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrEndFrame(this->Session, &frameEndInfo), "xrEndFrame failed");
      this->FrameBegan = false;
      return ok;
    }

    layer.layerFlags = (this->OptionalExtensions.RemotingSupported || this->FBPassthroughActive ||
                         this->EnvDepthActive)
      ? XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT
      : 0;

    layer.space = this->ReferenceSpace;
    layer.viewCount = viewCount;
    layer.views = this->RenderResources->ProjectionLayerViews.data();

    layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer));
  }

  XrFrameEndInfo frameEndInfo{ XR_TYPE_FRAME_END_INFO };
  frameEndInfo.displayTime = this->CurrentFrame.FrameState.predictedDisplayTime;
  frameEndInfo.environmentBlendMode = this->EnvironmentBlendMode;
  frameEndInfo.layerCount = static_cast<uint32_t>(layers.size());
  frameEndInfo.layers = layers.empty() ? nullptr : layers.data();

  // End frame (must always be called if BeginFrame succeeded)
  const bool ok = this->XrCheckOutput(
    vtkOpenXRManager::ErrorOutput, xrEndFrame(this->Session, &frameEndInfo), "xrEndFrame failed");

  // Always reset frame state even on failure (prevents deadlock of frame loop)
  this->FrameBegan = false;

  return ok;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::PollEvent(XrEventDataBuffer& eventData)
{
  eventData.type = XR_TYPE_EVENT_DATA_BUFFER;
  eventData.next = nullptr;
  return xrPollEvent(this->Instance, &eventData) == XR_SUCCESS;
}

//------------------------------------------------------------------------------
uint32_t vtkOpenXRManager::WaitAndAcquireSwapchainImage(const XrSwapchain& swapchainHandle)
{
  VTK_CHECK_NULL_XRHANDLE(
    swapchainHandle, "vtkOpenXRManager::WaitAndAcquireSwapchainImage, swapchain");

  uint32_t swapchainImageIndex;
  XrSwapchainImageAcquireInfo acquireInfo{ XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
  this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
    xrAcquireSwapchainImage(swapchainHandle, &acquireInfo, &swapchainImageIndex),
    "Failed to acquire swapchain image !");

  XrSwapchainImageWaitInfo waitInfo{ XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
  waitInfo.timeout = XR_INFINITE_DURATION;
  this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
    xrWaitSwapchainImage(swapchainHandle, &waitInfo), "Failed to wait swapchain image !");

  return swapchainImageIndex;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::XrCheckOutput(
  vtkOpenXRManager::OutputLevel level, const XrResult& result, const std::string& message)
{
  if (XR_FAILED(result))
  {
    char xRResultString[XR_MAX_RESULT_STRING_SIZE];
    xrResultToString(this->Instance, result, xRResultString);
    switch (level)
    {
      case vtkOpenXRManager::DebugOutput:
        vtkDebugWithObjectMacro(nullptr, << message << " [" << xRResultString << "].");
        break;
      case vtkOpenXRManager::WarningOutput:
        vtkWarningWithObjectMacro(nullptr, << message << " [" << xRResultString << "].");
        break;
      case vtkOpenXRManager::ErrorOutput:
        vtkErrorWithObjectMacro(nullptr, << message << " [" << xRResultString << "].");
        break;
      default:
        break;
    }
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::PrintInstanceProperties()
{
  XrInstanceProperties instanceProperties = {
    XR_TYPE_INSTANCE_PROPERTIES, // .type
    nullptr,                     // .next
  };

  this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
    xrGetInstanceProperties(this->Instance, &instanceProperties), "Failed to get instance info");

  std::cout << "Runtime Name: " << instanceProperties.runtimeName;
  std::cout << "Runtime Version: " << XR_VERSION_MAJOR(instanceProperties.runtimeVersion) << "."
            << XR_VERSION_MINOR(instanceProperties.runtimeVersion) << "."
            << XR_VERSION_PATCH(instanceProperties.runtimeVersion) << std::endl;
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::PrintSystemProperties(XrSystemProperties* systemProperties)
{
  std::cout << "System Properties for system id:" << systemProperties->systemId << ", with name \""
            << systemProperties->systemName << "\""
            << ", vendorID=" << systemProperties->vendorId << std::endl;

  std::cout << "\tMax Layers          : " << systemProperties->graphicsProperties.maxLayerCount
            << std::endl;
  std::cout << "\tMax Swapchain Height: "
            << systemProperties->graphicsProperties.maxSwapchainImageHeight << std::endl;
  std::cout << "\tMax Swapchain Width : "
            << systemProperties->graphicsProperties.maxSwapchainImageWidth << std::endl;
  std::cout << "\tOrientation Tracking: "
            << (systemProperties->trackingProperties.orientationTracking ? "True" : "False")
            << std::endl;
  std::cout << "\tPosition Tracking   : "
            << (systemProperties->trackingProperties.positionTracking ? "True" : "False")
            << std::endl;

  const XrBaseInStructure* next = static_cast<XrBaseInStructure*>(systemProperties->next);
  while (next)
  {
    if (next->type == XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT)
    {
      XrSystemHandTrackingPropertiesEXT* ht =
        static_cast<XrSystemHandTrackingPropertiesEXT*>(systemProperties->next);
      std::cout << "\tHand Tracking       : " << ht->supportsHandTracking << std::endl;
    }
    next = next->next;
  }
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::PrintSupportedViewConfigs()
{
  uint32_t viewConfigCount;
  this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
    xrEnumerateViewConfigurations(this->Instance, this->SystemId, 0, &viewConfigCount, nullptr),
    "Failed to get view configuration count");

  std::cout << "Runtime supports " << viewConfigCount << " view configurations" << std::endl;

  std::vector<XrViewConfigurationType> viewConfigs(viewConfigCount);
  this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
    xrEnumerateViewConfigurations(
      this->Instance, this->SystemId, viewConfigCount, &viewConfigCount, viewConfigs.data()),
    "Failed to enumerate view configurations!");

  for (uint32_t i = 0; i < viewConfigCount; ++i)
  {
    XrViewConfigurationProperties props = { XR_TYPE_VIEW_CONFIGURATION_PROPERTIES };
    this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
      xrGetViewConfigurationProperties(this->Instance, this->SystemId, viewConfigs[i], &props),
      "Failed to get view configuration info " + vtk::to_string(i));

    std::cout << "Type "
              << vtkOpenXRUtilities::GetViewConfigurationTypeAsString(props.viewConfigurationType)
              << ": FOV mutable: " << (props.fovMutable ? "True" : "False") << std::endl;
  }
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::PrintViewConfigViewInfo(
  const std::vector<XrViewConfigurationView>& viewconfigViews)
{
  for (size_t i = 0; i < viewconfigViews.size(); ++i)
  {
    const auto& vcfgv = viewconfigViews[i];
    std::cout << "View Configuration View " << i << std::endl;
    std::cout << "\tResolution       : Recommended: " << vcfgv.recommendedImageRectWidth << "x"
              << vcfgv.recommendedImageRectHeight << ", Max: " << vcfgv.maxImageRectWidth << "x"
              << vcfgv.maxImageRectHeight << std::endl;
    std::cout << "\tSwapchain Samples: Recommended: " << vcfgv.recommendedSwapchainSampleCount
              << ", Max: " << vcfgv.maxSwapchainSampleCount << std::endl;
  }
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::PrintReferenceSpaces()
{
  uint32_t refSpaceCount;
  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrEnumerateReferenceSpaces(this->Session, 0, &refSpaceCount, nullptr),
    "Getting number of reference spaces failed!");

  std::vector<XrReferenceSpaceType> refSpaces(refSpaceCount);
  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrEnumerateReferenceSpaces(this->Session, refSpaceCount, &refSpaceCount, refSpaces.data()),
    "Enumerating reference spaces failed!");

  std::cout << "Runtime supports " << refSpaceCount << " reference spaces:" << std::endl;
  for (uint32_t i = 0; i < refSpaceCount; i++)
  {
    if (refSpaces[i] == XR_REFERENCE_SPACE_TYPE_LOCAL)
    {
      std::cout << "\tXR_REFERENCE_SPACE_TYPE_LOCAL" << std::endl;
    }
    else if (refSpaces[i] == XR_REFERENCE_SPACE_TYPE_STAGE)
    {
      std::cout << "\tXR_REFERENCE_SPACE_TYPE_STAGE" << std::endl;
    }
    else if (refSpaces[i] == XR_REFERENCE_SPACE_TYPE_VIEW)
    {
      std::cout << "\tXR_REFERENCE_SPACE_TYPE_VIEW" << std::endl;
    }
    else
    {
      std::cout << "\tOther (extension?) refspace : " << refSpaces[i] << std::endl;
    }
  }

  return true;
}

//------------------------------------------------------------------------------
std::vector<const char*> vtkOpenXRManager::SelectExtensions(vtkOpenXRRenderWindow* window)
{
  // Fetch the list of extensions supported by the runtime.
  uint32_t extensionCount;
  bool countResult = this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrEnumerateInstanceExtensionProperties(nullptr, 0, &extensionCount, nullptr),
    "Failed to enumerate number of extension properties");

  std::vector<const char*> enabledExtensions;

  if (!countResult)
  {
    vtkWarningWithObjectMacro(nullptr, "Cannot select extensions, unable to get an accurate count");
    return enabledExtensions;
  }

  std::vector<XrExtensionProperties> extensionProperties(
    extensionCount, { XR_TYPE_EXTENSION_PROPERTIES });
  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrEnumerateInstanceExtensionProperties(
      nullptr, extensionCount, &extensionCount, extensionProperties.data()),
    "Failed to enumerate extension properties");

  // Add a specific extension to the list of extensions to be enabled, if it is supported.
  auto EnableExtensionIfSupported = [&](const char* extensionName)
  {
    for (uint32_t i = 0; i < extensionCount; i++)
    {
      if (strcmp(extensionProperties[i].extensionName, extensionName) == 0)
      {
        enabledExtensions.push_back(extensionName);
        return true;
      }
    }
    return false;
  };

  // Don't forget here to use the name of the extension (uppercase with suffix EXTENSION_NAME)
  this->RenderingBackendExtensionSupported =
    EnableExtensionIfSupported(this->GraphicsStrategy->GetBackendExtensionName());

  this->OptionalExtensions.ControllerModelExtensionSupported =
    EnableExtensionIfSupported(XR_MSFT_CONTROLLER_MODEL_EXTENSION_NAME);

  EnableExtensionIfSupported(XR_EXT_HP_MIXED_REALITY_CONTROLLER_EXTENSION_NAME);

  this->OptionalExtensions.UnboundedRefSpaceSupported =
    EnableExtensionIfSupported(XR_MSFT_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME);

  this->OptionalExtensions.SpatialAnchorSupported =
    EnableExtensionIfSupported(XR_MSFT_SPATIAL_ANCHOR_EXTENSION_NAME);

  this->OptionalExtensions.HandTrackingSupported =
    EnableExtensionIfSupported(XR_EXT_HAND_TRACKING_EXTENSION_NAME);

  this->OptionalExtensions.HandInteractionSupported =
    EnableExtensionIfSupported(XR_MSFT_HAND_INTERACTION_EXTENSION_NAME);

  this->OptionalExtensions.RemotingSupported =
    EnableExtensionIfSupported(this->ConnectionStrategy->GetExtensionName());

  if (window->GetUseDepthExtension())
  {
    this->OptionalExtensions.DepthExtensionSupported =
      EnableExtensionIfSupported(XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME);
  }

  if (window->GetEnableSceneUnderstanding())
  {
    this->OptionalExtensions.SceneUnderstandingSupported =
      EnableExtensionIfSupported(XR_MSFT_SCENE_UNDERSTANDING_EXTENSION_NAME);

    this->OptionalExtensions.SceneMarkerSupported =
      this->OptionalExtensions.SceneUnderstandingSupported &&
      EnableExtensionIfSupported(XR_MSFT_SCENE_MARKER_EXTENSION_NAME);
  }

  if (window->GetUsePassthrough())
  {
    this->OptionalExtensions.FBPassthroughSupported =
      EnableExtensionIfSupported(XR_FB_PASSTHROUGH_EXTENSION_NAME);

    // If XR_FB_passthrough is available, use OPAQUE blend mode — the camera feed
    // is composited via a dedicated composition layer in EndFrame().  Only keep the
    // ALPHA_BLEND preference as a fallback for runtimes that support it but not
    // XR_FB_passthrough.
    if (this->OptionalExtensions.FBPassthroughSupported)
    {
      this->PreferredEnvironmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_MAX_ENUM;
    }
    else
    {
      // Log all extensions the runtime offers to aid diagnosis when passthrough
      // is unavailable (e.g. Meta Quest Link requires the headset to have
      // developer mode enabled and the Meta XR PC app to grant access).
      std::string available;
      for (uint32_t i = 0; i < extensionCount; i++)
      {
        if (i > 0)
        {
          available += ", ";
        }
        available += extensionProperties[i].extensionName;
      }
      vtkWarningWithObjectMacro(
        nullptr, << "XR_FB_passthrough (\"" << XR_FB_PASSTHROUGH_EXTENSION_NAME
                 << "\") is not in the runtime's extension list. "
                 << "For Meta Quest Link, ensure developer mode is enabled on the headset "
                 << "and passthrough access is granted in the Meta XR PC app settings. "
                 << "Runtime offers " << extensionCount << " extension(s): " << available);
    }

#ifdef XR_META_environment_depth
    // Only enable environment depth when real-world depth occlusion will actually be used
    // (OccludedOpacity < 1.0).  Loading the extension when unused causes
    // xrAcquireEnvironmentDepthImageMETA to run every frame (via BeginSession ->
    // StartEnvironmentDepth -> WaitAndBeginFrame -> AcquireEnvironmentDepthImage),
    // which blocks the render loop and produces position flickering.
    if (window->GetOccludedOpacity() < 1.0f || window->GetCaptureEnvironmentDepth())
    {
      this->OptionalExtensions.EnvironmentDepthSupported =
        EnableExtensionIfSupported(XR_META_ENVIRONMENT_DEPTH_EXTENSION_NAME);
      if (!this->OptionalExtensions.EnvironmentDepthSupported)
      {
        vtkDebugWithObjectMacro(nullptr,
          "XR_META_environment_depth is not available; real-world depth occlusion disabled.");
      }
    }
#endif
  }

  this->PrintOptionalExtensions();

  return enabledExtensions;
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::PrintOptionalExtensions()
{
  if (this->OptionalExtensions.DepthExtensionSupported)
  {
    std::cout << "Optional extensions DepthExtension is supported" << std::endl;
  }
  if (this->OptionalExtensions.ControllerModelExtensionSupported)
  {
    std::cout << "Optional extensions ControllerModelExtension is supported" << std::endl;
  }
  if (this->OptionalExtensions.UnboundedRefSpaceSupported)
  {
    std::cout << "Optional extensions UnboundedRefSpace is supported" << std::endl;
  }
  if (this->OptionalExtensions.SpatialAnchorSupported)
  {
    std::cout << "Optional extensions SpatialAnchor is supported" << std::endl;
  }
  if (this->OptionalExtensions.HandTrackingSupported)
  {
    std::cout << "Optional extensions HandTracking is supported" << std::endl;
  }
  if (this->OptionalExtensions.HandInteractionSupported)
  {
    std::cout << "Optional extensions HandInteraction is supported" << std::endl;
  }
  if (this->OptionalExtensions.RemotingSupported)
  {
    std::cout << "Optional extensions Remoting is supported" << std::endl;
  }
  if (this->OptionalExtensions.SceneUnderstandingSupported)
  {
    std::cout << "Optional extensions Scene Understanding is supported" << std::endl;
  }
  if (this->OptionalExtensions.SceneMarkerSupported)
  {
    std::cout << "Optional extensions Scene Marker is supported" << std::endl;
  }
  if (this->OptionalExtensions.FBPassthroughSupported)
  {
    std::cout << "Optional extensions XR_FB_passthrough is supported" << std::endl;
  }
  if (this->OptionalExtensions.EnvironmentDepthSupported)
  {
    std::cout << "Optional extensions XR_META_environment_depth is supported" << std::endl;
  }
}

//------------------------------------------------------------------------------
// Instance and extensions
//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateInstance(vtkOpenXRRenderWindow* window)
{
  // Start by selection available extensions
  const std::vector<const char*> enabledExtensions = this->SelectExtensions(window);

  // Check that the requested rendering backend is supported
  if (!this->RenderingBackendExtensionSupported)
  {
    vtkErrorWithObjectMacro(nullptr, << "Rendering backend extension is not supported. Aborting.");
    return false;
  }

  // Create the instance with enabled extensions.
  XrInstanceCreateInfo createInfo{ XR_TYPE_INSTANCE_CREATE_INFO };
  createInfo.enabledExtensionCount = (uint32_t)enabledExtensions.size();
  createInfo.enabledExtensionNames = enabledExtensions.data();

  XrApplicationInfo applicationInfo = {
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

  createInfo.applicationInfo = applicationInfo;

  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrCreateInstance(&createInfo, &this->Instance), "Failed to create XR instance."))
  {
    return false;
  }

  this->PrintInstanceProperties();

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateSubactionPaths()
{
  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrStringToPath(this->Instance, "/user/hand/left",
          &this->SubactionPaths[vtkOpenXRManager::ControllerIndex::Left]),
        "Failed to create left hand subaction path"))
  {
    return false;
  }
  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrStringToPath(this->Instance, "/user/hand/right",
          &this->SubactionPaths[vtkOpenXRManager::ControllerIndex::Right]),
        "Failed to create right hand subaction path"))
  {
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
// System
//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateSystem()
{
  VTK_CHECK_NULL_XRHANDLE(this->Instance, "vtkOpenXRManager::CreateSystem, Instance");

  // --- Create XrSystem
  XrSystemGetInfo system_get_info = {
    XR_TYPE_SYSTEM_GET_INFO, // .type
    nullptr,                 // .next
    this->FormFactor,        // .formFactor
  };

  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrGetSystem(this->Instance, &system_get_info, &this->SystemId),
    "Failed to get system for HMD form factor.");

  vtkDebugWithObjectMacro(
    nullptr, "Successfully got XrSystem with id " << this->SystemId << " for HMD form factor.");

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateSystemProperties()
{
  // checking system properties is generally optional, but we are interested in hand tracking
  // support
  {
    XrSystemProperties systemProperties = {
      XR_TYPE_SYSTEM_PROPERTIES, // .type
      nullptr,                   // .next
      XR_NULL_SYSTEM_ID,         // .systemId
      0,                         // .vendorId
      "",                        // .systemName
      { 0 },                     // .graphicsProperties,
      { 0 }                      // .trackingProperties,
    };

    XrSystemHandTrackingPropertiesEXT ht = {
      XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT, // .type
      nullptr,                                     // .next
      false                                        // .supportsHandTracking
    };

    if (this->OptionalExtensions.HandTrackingSupported)
    {
      systemProperties.next = &ht;
    }

    this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
      xrGetSystemProperties(this->Instance, this->SystemId, &systemProperties),
      "Failed to get System properties");

    this->OptionalExtensions.HandTrackingSupported =
      this->OptionalExtensions.HandTrackingSupported && ht.supportsHandTracking;

    this->PrintSystemProperties(&systemProperties);
  }

  // Choose an environment blend mode
  {
    // Query the list of supported environment blend modes for the current system
    uint32_t count;

    xrEnumerateEnvironmentBlendModes(
      this->Instance, this->SystemId, this->ViewType, 0, &count, nullptr);
    this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
      xrEnumerateEnvironmentBlendModes(
        this->Instance, this->SystemId, this->ViewType, 0, &count, nullptr),
      "Failed to get environment blend modes count");
    if (count == 0)
    {
      vtkErrorWithObjectMacro(
        nullptr, "A system must support at least one environment blend mode.");
    }

    std::vector<XrEnvironmentBlendMode> environmentBlendModes(count);
    xrEnumerateEnvironmentBlendModes(
      this->Instance, this->SystemId, this->ViewType, count, &count, environmentBlendModes.data());
    this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
      xrEnumerateEnvironmentBlendModes(this->Instance, this->SystemId, this->ViewType, count,
        &count, environmentBlendModes.data()),
      "Failed to enumerate environment blend modes");

    // If a preferred blend mode was requested and the runtime supports it, use it.
    // Otherwise fall back to the runtime's first (most preferred) mode.
    this->EnvironmentBlendMode = environmentBlendModes[0];
    if (this->PreferredEnvironmentBlendMode != XR_ENVIRONMENT_BLEND_MODE_MAX_ENUM)
    {
      for (const XrEnvironmentBlendMode& mode : environmentBlendModes)
      {
        if (mode == this->PreferredEnvironmentBlendMode)
        {
          this->EnvironmentBlendMode = mode;
          break;
        }
      }
      if (this->EnvironmentBlendMode != this->PreferredEnvironmentBlendMode)
      {
        vtkWarningWithObjectMacro(nullptr,
          "Requested environment blend mode is not supported by the runtime. "
          "Falling back to the runtime's preferred blend mode.");
      }
    }
  }

  this->PrintSupportedViewConfigs();

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateSession()
{
  VTK_CHECK_NULL_XRHANDLE(this->Instance, "vtkOpenXRManager::CreateSession, Instance");

  // --- Create session
  this->SessionState = XR_SESSION_STATE_UNKNOWN;

  XrSessionCreateInfo sessionCreateInfo = {
    XR_TYPE_SESSION_CREATE_INFO,                  // .type
    this->GraphicsStrategy->GetGraphicsBinding(), // .next
    0,                                            // .createFlags
    this->SystemId                                // .systemId
  };

  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrCreateSession(this->Instance, &sessionCreateInfo, &this->Session),
        "Failed to create session"))
  {
    return false;
  }

#ifdef XR_USE_GRAPHICS_API_OPENGL
  vtkDebugWithObjectMacro(nullptr, "Successfully created a session with OpenGL!");
#elif XR_USE_GRAPHICS_API_D3D11
  vtkDebugWithObjectMacro(nullptr, "Successfully created a session with DirectX!");
#endif

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateReferenceSpace()
{
  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::CreateReferenceSpace, Session");

  // Many runtimes support at least STAGE and LOCAL but not all do.
  // Sophisticated apps might check if the chosen one is supported and try another one if not.
  // Here we will get an error from xrCreateReferenceSpace() and exit.
  if (!this->PrintReferenceSpaces())
  {
    return false;
  }

  // Choose an unbounded reference space to improve holographic remoting stability
  if (this->OptionalExtensions.RemotingSupported &&
    this->OptionalExtensions.UnboundedRefSpaceSupported)
  {
    this->ReferenceSpaceType = XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT;
  }

  XrReferenceSpaceCreateInfo refSpaceCreateInfo = {
    XR_TYPE_REFERENCE_SPACE_CREATE_INFO,  // .type
    nullptr,                              // .next
    this->ReferenceSpaceType,             // .referenceSpaceType
    vtkOpenXRUtilities::GetIdentityPose() // .poseInReferenceSpace
  };

  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrCreateReferenceSpace(this->Session, &refSpaceCreateInfo, &this->ReferenceSpace),
    "Failed to create play space!");

  return true;
}

//------------------------------------------------------------------------------
std::tuple<int64_t, int64_t> vtkOpenXRManager::SelectSwapchainPixelFormats()
{
  // Query the runtime's preferred swapchain formats.
  uint32_t swapchainFormatsCount;
  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrEnumerateSwapchainFormats(this->Session, 0, &swapchainFormatsCount, nullptr),
    "Failed to get number of supported swapchain formats");

  vtkDebugWithObjectMacro(
    nullptr, "Runtime supports " << swapchainFormatsCount << " swapchain formats");

  std::vector<int64_t> swapchainFormats(swapchainFormatsCount);
  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrEnumerateSwapchainFormats(
      this->Session, swapchainFormatsCount, &swapchainFormatsCount, swapchainFormats.data()),
    "Failed to enumerate swapchain formats");

  // Choose the first runtime-preferred format that this app supports.
  auto selectPixelFormat = [&](const std::vector<int64_t>& runtimePreferredFormats,
                             const std::vector<int64_t>& applicationSupportedFormats,
                             const std::string& formatName)
  {
    auto found =
      std::find_first_of(std::begin(runtimePreferredFormats), std::end(runtimePreferredFormats),
        std::begin(applicationSupportedFormats), std::end(applicationSupportedFormats));
    if (found == std::end(runtimePreferredFormats))
    {
      vtkErrorWithObjectMacro(
        nullptr, << "No runtime swapchain " << formatName << " format in the list is supported.");
      return (int64_t)-1;
    }
    return *found;
  };

  int64_t colorSwapchainFormat = selectPixelFormat(
    swapchainFormats, this->GraphicsStrategy->GetSupportedColorFormats(), "color");
  int64_t depthSwapchainFormat = -1;
  if (this->OptionalExtensions.DepthExtensionSupported)
  {
    depthSwapchainFormat = selectPixelFormat(
      swapchainFormats, this->GraphicsStrategy->GetSupportedDepthFormats(), "depth");
    if (depthSwapchainFormat == -1)
    {
      vtkDebugWithObjectMacro(
        nullptr, "Disabling depth extension as no depth format are supported");
      this->OptionalExtensions.DepthExtensionSupported = false;
    }
  }

  return std::make_tuple(colorSwapchainFormat, depthSwapchainFormat);
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateSwapchains()
{
  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::CreateSwapchains, Session");

  this->RenderResources = std::unique_ptr<RenderResources_t>(new RenderResources_t());

  // Select color and depth swapchain pixel formats.
  int64_t colorSwapchainFormat, depthSwapchainFormat;
  std::tie(colorSwapchainFormat, depthSwapchainFormat) = this->SelectSwapchainPixelFormats();

  // Query and cache view configuration views.
  this->CreateConfigViews();

  const XrViewConfigurationView& view = this->RenderResources->ConfigViews[0];

  // Use the system's recommended rendering parameters.
  const uint32_t imageRectWidth = view.recommendedImageRectWidth;
  const uint32_t imageRectHeight = view.recommendedImageRectHeight;
  const uint32_t swapchainSampleCount = view.recommendedSwapchainSampleCount;

  // Create swapchains with texture array for color and depth images.
  const uint32_t viewCount = (uint32_t)this->RenderResources->ConfigViews.size();

  // One swapchain per view to make it simple
  // We could also use a texture arraySize != 1 but the rendering
  // will be more complex
  this->RenderResources->ColorSwapchains.resize(viewCount);
  this->RenderResources->DepthSwapchains.resize(viewCount);

  this->GraphicsStrategy->SetNumberOfSwapchains(viewCount);

  for (uint32_t i = 0; i < viewCount; ++i)
  {
    this->RenderResources->ColorSwapchains[i] = this->CreateSwapchain(colorSwapchainFormat,
      imageRectWidth, imageRectHeight, swapchainSampleCount, 0 /*createFlags*/,
      XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT);
    this->GraphicsStrategy->EnumerateColorSwapchainImages(
      this->RenderResources->ColorSwapchains[i].Swapchain, i);

    if (this->OptionalExtensions.DepthExtensionSupported)
    {
      this->RenderResources->DepthSwapchains[i] = this->CreateSwapchain(depthSwapchainFormat,
        imageRectWidth, imageRectHeight, swapchainSampleCount, 0 /*createFlags*/,
        XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT | XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
      this->GraphicsStrategy->EnumerateDepthSwapchainImages(
        this->RenderResources->DepthSwapchains[i].Swapchain, i);
    }
  }

  // Preallocate view buffers for xrLocateViews later inside frame loop.
  this->CurrentFrame.Views.resize(viewCount, { XR_TYPE_VIEW });

  // Preallocate projection layer views and depth if needed
  this->RenderResources->ProjectionLayerViews.resize(viewCount);
  if (this->OptionalExtensions.DepthExtensionSupported)
  {
    this->RenderResources->DepthInfoViews.resize(viewCount);
  }

  return true;
}

//------------------------------------------------------------------------------
vtkOpenXRManager::Swapchain_t vtkOpenXRManager::CreateSwapchain(int64_t format, uint32_t width,
  uint32_t height, uint32_t sampleCount, XrSwapchainCreateFlags createFlags,
  XrSwapchainUsageFlags usageFlags)
{
  Swapchain_t swapchain;
  swapchain.Format = format;
  swapchain.Width = width;
  swapchain.Height = height;

  XrSwapchainCreateInfo swapchainCreateInfo{ XR_TYPE_SWAPCHAIN_CREATE_INFO };
  swapchainCreateInfo.arraySize = 1;
  swapchainCreateInfo.format = format;
  swapchainCreateInfo.width = width;
  swapchainCreateInfo.height = height;
  swapchainCreateInfo.mipCount = 1;
  swapchainCreateInfo.faceCount = 1;
  swapchainCreateInfo.sampleCount = sampleCount;
  swapchainCreateInfo.createFlags = createFlags;
  swapchainCreateInfo.usageFlags = usageFlags;

  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrCreateSwapchain(this->Session, &swapchainCreateInfo, &swapchain.Swapchain),
    "Failed to create swapchain!");

  return swapchain;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateConfigViews()
{
  uint32_t viewCount;
  this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrEnumerateViewConfigurationViews(
      this->Instance, this->SystemId, this->ViewType, 0, &viewCount, nullptr),
    "Failed to get view configuration view count!");
  if (viewCount != this->StereoViewCount)
  {
    vtkWarningWithObjectMacro(nullptr, << "StereoViewCount (" << this->StereoViewCount
                                       << ") is different than viewCount (" << viewCount << ")");
  }

  this->RenderResources->ConfigViews.resize(viewCount, { XR_TYPE_VIEW_CONFIGURATION_VIEW });

  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrEnumerateViewConfigurationViews(this->Instance, this->SystemId, this->ViewType, viewCount,
          &viewCount, this->RenderResources->ConfigViews.data()),
        "Failed to enumerate view configuration views!"))
  {
    return false;
  }

  this->PrintViewConfigViewInfo(this->RenderResources->ConfigViews);

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateActionSet(
  const std::string& actionSetName, const std::string& localizedActionSetName)
{
  vtkDebugWithObjectMacro(
    nullptr, "Create action set " << actionSetName << ": " << localizedActionSetName);

  XrActionSetCreateInfo actionSetInfo{ XR_TYPE_ACTION_SET_CREATE_INFO };

  strcpy(actionSetInfo.actionSetName, actionSetName.c_str());
  strcpy(actionSetInfo.localizedActionSetName, localizedActionSetName.c_str());

  XrActionSet actionSet;
  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrCreateActionSet(this->Instance, &actionSetInfo, &actionSet),
        "Failed to create default actionset"))
  {
    return false;
  }
  this->ActionSets.push_back(actionSet);

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::SelectActiveActionSet(unsigned int index)
{
  if (this->ActionSets.empty())
  {
    vtkErrorWithObjectMacro(nullptr, << "An action set must be created prior to select one.");
    return false;
  }
  if (index >= this->ActionSets.size())
  {
    vtkWarningWithObjectMacro(nullptr,
      << "The selected action set at index : " << index << " does not exist. Pick the first one");
    index = 0;
  }

  this->ActiveActionSet = &this->ActionSets[index];

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::AttachSessionActionSets()
{
  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::AttachSessionActionSets, Session");

  XrSessionActionSetsAttachInfo actionSetsAttachInfo = {
    XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO, // .type
    nullptr,                                 // .next
    (uint32_t)this->ActionSets.size(),       // .countActionSets
    this->ActionSets.data()                  // .actionSets
  };
  return this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrAttachSessionActionSets(this->Session, &actionSetsAttachInfo),
    "Failed to attach action sets");
}

//------------------------------------------------------------------------------
void vtkOpenXRManager::DestroyActionSets()
{
  for (XrActionSet actionSet : this->ActionSets)
  {
    xrDestroyActionSet(actionSet);
  }

  this->ActionSets.clear();

  // active action set pointed to one of those, so clear it now
  this->ActiveActionSet = nullptr;
}

//------------------------------------------------------------------------------
XrPath vtkOpenXRManager::GetXrPath(const std::string& path)
{
  VTK_CHECK_NULL_XRHANDLE(this->Instance, "vtkOpenXRManager::GetXrPath, Instance");

  XrPath xrPath;
  this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
    xrStringToPath(this->Instance, path.c_str(), &xrPath), "Failed to get path " + path);
  return xrPath;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateOneAction(
  Action_t& actionT, const std::string& name, const std::string& localizedName)
{
  if (this->ActiveActionSet == nullptr)
  {
    return false;
  }

  XrActionCreateInfo actionInfo = {
    XR_TYPE_ACTION_CREATE_INFO,            // .type
    nullptr,                               // .next
    "",                                    // .actionName
    actionT.ActionType,                    // .actionType
    (uint32_t)this->SubactionPaths.size(), // .countSubactionPaths
    this->SubactionPaths.data(),           // .subactionPaths
    ""                                     // .localizedActionName
  };
  strcpy(actionInfo.actionName, name.c_str());
  strcpy(actionInfo.localizedActionName, localizedName.c_str());

  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrCreateAction(*this->ActiveActionSet, &actionInfo, &actionT.Action),
        "Failed to create action " + std::string(name)))
  {
    return false;
  }

  // Pose action spaces are intentionally NOT created here. The Meta Quest
  // runtime requires xrAttachSessionActionSets to have been called before
  // xrCreateActionSpace; otherwise xrLocateSpace returns locationFlags=0
  // forever even though the underlying action becomes active and the
  // interaction profile binds correctly. The interactor is responsible for
  // calling CreateActionPoseSpaces() on every pose action AFTER
  // AttachSessionActionSets().

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateActionPoseSpaces(Action_t& actionT)
{
  if (actionT.ActionType != XR_ACTION_TYPE_POSE_INPUT)
  {
    return true;
  }
  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::CreateActionPoseSpaces, Session");
  VTK_CHECK_NULL_XRHANDLE(actionT.Action, "vtkOpenXRManager::CreateActionPoseSpaces, Action");

  for (uint32_t hand :
    { vtkOpenXRManager::ControllerIndex::Left, vtkOpenXRManager::ControllerIndex::Right })
  {
    if (!this->CreateOneActionSpace(actionT.Action, this->SubactionPaths[hand],
          vtkOpenXRUtilities::GetIdentityPose(), actionT.PoseSpaces[hand]))
    {
      vtkErrorWithObjectMacro(nullptr,
        << "Failed to create pose action space for "
        << (hand == vtkOpenXRManager::ControllerIndex::Left ? "left" : "right") << " hand");
      return false;
    }
  }
  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::CreateOneActionSpace(const XrAction& action, const XrPath& subactionPath,
  const XrPosef& poseInActionSpace, XrSpace& space)
{
  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::CreateOneActionSpace, Session");

  XrActionSpaceCreateInfo actionSpaceInfo = {
    XR_TYPE_ACTION_SPACE_CREATE_INFO, // .type
    nullptr,                          // .next
    action,                           // .action
    subactionPath,                    // .subactionPath
    poseInActionSpace                 // .poseInActionSpace
  };

  return this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
    xrCreateActionSpace(this->Session, &actionSpaceInfo, &space), "");
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::SuggestActions(
  const std::string& profile, std::vector<XrActionSuggestedBinding>& actionSuggestedBindings)
{
  vtkDebugWithObjectMacro(nullptr, "SuggestActions for profile : " << profile);
  VTK_CHECK_NULL_XRHANDLE(this->Instance, "vtkOpenXRManager::SuggestActions, Instance");

  XrPath interactionProfilePath;
  this->XrCheckOutput(vtkOpenXRManager::WarningOutput,
    xrStringToPath(this->Instance, profile.c_str(), &interactionProfilePath),
    "Failed to get interaction profile path " + profile);

  const XrInteractionProfileSuggestedBinding suggestedBindings = {
    XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING, // .type
    nullptr,                                       // .next
    interactionProfilePath,                        // .interactionProfile
    (uint32_t)actionSuggestedBindings.size(),      // .countSuggestedBindings
    actionSuggestedBindings.data()                 // .suggestedBindings
  };

  this->XrCheckOutput(vtkOpenXRManager::DebugOutput,
    xrSuggestInteractionProfileBindings(this->Instance, &suggestedBindings),
    "Could not suggest actions");

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::SyncActions()
{
  if (this->ActiveActionSet == nullptr)
  {
    return false;
  }
  const XrActionSet& actionSet = *this->ActiveActionSet;
  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::SyncActions, Session");
  VTK_CHECK_NULL_XRHANDLE(actionSet, "vtkOpenXRManager::SyncActions, ActiveActionSet");

  // Only use the active action set, but we could add all action sets
  // in the following vector
  std::vector<XrActiveActionSet> activeActionSets = { { actionSet, XR_NULL_PATH } };
  XrActionsSyncInfo syncInfo{ XR_TYPE_ACTIONS_SYNC_INFO };
  syncInfo.countActiveActionSets = (uint32_t)activeActionSets.size();
  syncInfo.activeActionSets = activeActionSets.data();
  return this->XrCheckOutput(vtkOpenXRManager::ErrorOutput, xrSyncActions(this->Session, &syncInfo),
    "Failed to sync actions");
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::UpdateActionData(Action_t& action_t, const int hand)
{
  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::UpdateActionData, Session");
  VTK_CHECK_NULL_XRHANDLE(
    this->ReferenceSpace, "vtkOpenXRManager::UpdateActionData, ReferenceSpace");

  XrActionStateGetInfo info = {
    XR_TYPE_ACTION_STATE_GET_INFO, // .type
    nullptr,                       // .next
    action_t.Action,               // .action
    this->SubactionPaths[hand],    // .subactionPath
  };

  // we store the state of the action, depending on the selected hand
  switch (action_t.ActionType)
  {
    case XR_ACTION_TYPE_FLOAT_INPUT:
      action_t.States[hand]._float.type = XR_TYPE_ACTION_STATE_FLOAT;
      action_t.States[hand]._float.next = nullptr;
      if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
            xrGetActionStateFloat(Session, &info, &action_t.States[hand]._float),
            "Failed to get float value"))
      {
        return false;
      }
      break;
    case XR_ACTION_TYPE_BOOLEAN_INPUT:
      action_t.States[hand]._boolean.type = XR_TYPE_ACTION_STATE_BOOLEAN;
      action_t.States[hand]._boolean.next = nullptr;
      if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
            xrGetActionStateBoolean(this->Session, &info, &action_t.States[hand]._boolean),
            "Failed to get boolean value"))
      {
        return false;
      }
      break;
    case XR_ACTION_TYPE_VECTOR2F_INPUT:
      action_t.States[hand]._vec2f.type = XR_TYPE_ACTION_STATE_VECTOR2F;
      action_t.States[hand]._vec2f.next = nullptr;
      if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
            xrGetActionStateVector2f(this->Session, &info, &action_t.States[hand]._vec2f),
            "Failed to get vec2f"))
      {
        return false;
      }
      break;
    case XR_ACTION_TYPE_POSE_INPUT:
      action_t.States[hand]._pose.type = XR_TYPE_ACTION_STATE_POSE;
      action_t.States[hand]._pose.next = nullptr;
      if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
            xrGetActionStatePose(this->Session, &info, &action_t.States[hand]._pose),
            "Failed to get action state pose"))
      {
        return false;
      }

      // Always locate the space to get fresh locationFlags, regardless of isActive.
      // When isActive is false the pose may be unreliable, but the locationFlags
      // returned by xrLocateSpace will reflect that (bits will not be set), so
      // callers that check locationFlags before using the pose are safe.
      // Gating xrLocateSpace on isActive left locationFlags stale (or zero at
      // startup), causing valid-pose checks downstream to fail even when the
      // runtime could still supply a usable pose.
      {
        action_t.PoseLocations[hand].type = XR_TYPE_SPACE_LOCATION;
        action_t.PoseLocations[hand].next = nullptr;

        if (this->StorePoseVelocities)
        {
          action_t.PoseVelocities[hand].type = XR_TYPE_SPACE_VELOCITY;
          action_t.PoseVelocities[hand].next = nullptr;
          action_t.PoseLocations[hand].next = &action_t.PoseVelocities[hand];
        }

        // Store the position of the hand
        if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
              xrLocateSpace(action_t.PoseSpaces[hand], this->ReferenceSpace,
                this->CurrentFrame.FrameState.predictedDisplayTime, &action_t.PoseLocations[hand]),
              "Failed to locate hand space"))
        {
          return false;
        }
      }

      break;
    default:
      break;
  }

  return true;
}

//------------------------------------------------------------------------------
bool vtkOpenXRManager::ApplyVibration(const Action_t& actionT, const int hand,
  const float amplitude, const float duration, const float frequency)
{
  VTK_CHECK_NULL_XRHANDLE(this->Session, "vtkOpenXRManager::ApplyVibration, Session");

  if (actionT.ActionType != XR_ACTION_TYPE_VIBRATION_OUTPUT)
  {
    vtkErrorWithObjectMacro(
      nullptr, << "vtkOpenXRManager::ApplyVibration must be called for an action of type "
                  "XR_ACTION_TYPE_VIBRATION_OUTPUT, not a "
               << vtkOpenXRUtilities::GetActionTypeAsString(actionT.ActionType));
    return false;
  }

  XrHapticActionInfo actionInfo{ XR_TYPE_HAPTIC_ACTION_INFO };
  actionInfo.action = actionT.Action;
  actionInfo.subactionPath = this->SubactionPaths[hand];

  XrHapticVibration vibration{ XR_TYPE_HAPTIC_VIBRATION };
  vibration.amplitude = amplitude;
  vibration.duration = duration;
  vibration.frequency = frequency;

  if (!this->XrCheckOutput(vtkOpenXRManager::ErrorOutput,
        xrApplyHapticFeedback(this->Session, &actionInfo, (XrHapticBaseHeader*)&vibration),
        "Failed to apply haptic feedback"))
  {
    return false;
  }
  return true;
}

VTK_ABI_NAMESPACE_END
} // namespace vtk::detail
