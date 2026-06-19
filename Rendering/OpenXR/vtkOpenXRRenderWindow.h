// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkOpenXRRenderWindow
 * @brief   OpenXR rendering window
 *
 *
 * vtkOpenXRRenderWindow is a concrete implementation of the abstract
 * class vtkRenderWindow.
 *
 * This class handles the bulk of interfacing to OpenXR. It supports one
 * renderer currently. The renderer is assumed to cover the entire window
 * which is what makes sense to VR. Overlay renderers can probably be
 * made to work with this but consider how overlays will appear in a
 * HMD if they do not track the viewpoint etc.
 *
 * OpenXR provides HMD and controller positions in "Physical" coordinate
 * system.
 * Origin: user's eye position at the time of calibration.
 * Axis directions: x = user's right; y = user's up; z = user's back.
 * Unit: meter.
 *
 * Renderer shows actors in World coordinate system. Transformation between
 * Physical and World coordinate systems is defined by PhysicalToWorldMatrix.
 * This matrix determines the user's position and orientation in the rendered
 * scene and scaling (magnification) of rendered actors.
 *
 */

#ifndef vtkOpenXRRenderWindow_h
#define vtkOpenXRRenderWindow_h

#include "vtkRenderingOpenXRModule.h" // For export macro
#include "vtkVRRenderWindow.h"

#include "vtkEventData.h" // for method sig
#include "vtkOpenXR.h"   // for XR_META_environment_depth and OpenXR types

#include <array>   // array
#include <cstdint> // uint32_t
#include <memory>  // unique_ptr
#include <string>  // string

// Forward declaration for environment depth occlusion pre-pass (used when
// XR_META_environment_depth is available).
#ifdef XR_META_environment_depth
class vtkOpenXREnvironmentDepthOcclusionPrePass;
#endif

VTK_ABI_NAMESPACE_BEGIN

class vtkMatrix4x4;
class vtkOpenXRSceneObserver;
class vtkOpenXRManagerConnection;

class VTKRENDERINGOPENXR_EXPORT vtkOpenXRRenderWindow : public vtkVRRenderWindow
{
public:
  static vtkOpenXRRenderWindow* New();
  vtkTypeMacro(vtkOpenXRRenderWindow, vtkVRRenderWindow);

  /**
   * Create an interactor to control renderers in this window.
   */
  VTK_NEWINSTANCE vtkRenderWindowInteractor* MakeRenderWindowInteractor() override;

  /**
   * Add a renderer to the list of renderers.
   */
  void AddRenderer(vtkRenderer*) override;

  /**
   * Update the system, if needed, due to stereo rendering. For some stereo
   * methods, subclasses might need to switch some hardware settings here.
   */
  void StereoUpdate() override;

  /**
   * Intermediate method performs operations required between the rendering
   * of the left and right eye.
   */
  void StereoMidpoint() override;

  /**
   * Handles work required once both views have been rendered when using
   * stereo rendering.
   */
  void StereoRenderComplete() override;

  /**
   * Overridden to not release resources that would interfere with an external
   * application's rendering. Avoiding round trip.
   */
  void Render() override;

  /**
   * Initialize the rendering window.  This will setup all system-specific
   * resources.  This method and Finalize() must be symmetric and it
   * should be possible to call them multiple times, even changing WindowId
   * in-between.  This is what WindowRemap does.
   */
  void Initialize() override;

  /**
   * Finalize the rendering window.  This will shutdown all system-specific
   * resources.  After having called this, it should be possible to destroy
   * a window that was used for a SetWindowId() call without any ill effects.
   */
  void Finalize() override;

  /**
   * Get report of capabilities for the render window
   */
  const char* ReportCapabilities() override { return "OpenXR System"; }

  /**
   * Get size of render window from OpenXR.
   */
  bool GetSizeFromAPI() override;

  /**
   * Check to see if a mouse button has been pressed or mouse wheel activated.
   * All other events are ignored by this method.
   * Maybe should return 1 always?
   */
  vtkTypeBool GetEventPending() override { return 0; }

  /**
   * Set the active state (active: true / inactive: false) of the specified hand.
   */
  void SetModelActiveState(const int hand, bool state) { this->ModelsActiveState[hand] = state; }

  uint32_t GetDeviceHandleForOpenXRHandle(uint32_t index);
  vtkEventDataDevice GetDeviceForOpenXRHandle(uint32_t ohandle);

  /**
   * Update the HMD pose based on hardware pose and physical to world transform.
   * VR camera properties are directly modified based on physical to world to
   * simulate \sa PhysicalTranslation, \sa PhysicalScale, etc.
   */
  void UpdateHMDMatrixPose() override;

  /**
   * Render the controllers
   */
  void RenderModels() override;

  /**
   * Get/Set the current interaction profile for a hand
   */
  std::string& GetCurrentInteractionProfile(uint32_t);
  void SetCurrentInteractionProfile(uint32_t, const std::string& profile);

  ///@{
  /**
   * Get/Set a custom path to look for the controllers models.
   * Default is empty, only the parent of the directory containing the library/executable is
   * checked.
   */
  std::string& GetModelsManifestDirectory();
  void SetModelsManifestDirectory(const std::string& path);
  ///@}

  ///@{
  /**
   * Enable or disable `XR_KHR_composition_layer_depth` extension when available.
   *
   * This must be set before initializing the render window.
   * If the extension is unavailable, this setting has no effect.
   *
   * Depth information enhances augmented reality experiences, particularly on
   * devices like the HoloLens 2. When enabled and the extension is available,
   * the render window's depth texture is submitted to the runtime, improving
   * hologram stability on supported devices.
   *
   * \note Enabling this option when no depth information are available could reduce stability.
   * This will be the case when translucent actors are in the scene.
   * When partial depth information are available,
   * which may happen when having translucent and opaque actors in the scene,
   * stability could be reduced **and** translucent actors may be distorded.
   *
   * Default value: `false`
   */
  vtkSetMacro(UseDepthExtension, bool);
  vtkGetMacro(UseDepthExtension, bool);
  ///@}

  ///@{
  /**
   * Enable or disable `XR_MSFT_scene_understanding` extension when available.
   *
   * This must be set before initializing the render window.
   * If the extension is unavailable, this setting has no effect.
   *
   * When enabled, creates a `vtkOpenXRScene`,
   * which can be accessed via `GetSceneObserver()`.
   *
   * Default value: `false`
   *
   * \sa GetSceneObserver
   */
  vtkSetMacro(EnableSceneUnderstanding, bool);
  vtkGetMacro(EnableSceneUnderstanding, bool);
  ///@}

  ///@{
  /**
   * Enable or disable camera passthrough (mixed reality / AR mode).
   *
   * When enabled, the OpenXR manager is requested to use
   * `XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND` so that real-world content seen
   * through the headset's cameras shows through wherever the rendered image
   * has a transparent (alpha = 0) background.
   *
   * This must be set before initializing the render window.
   * If the runtime does not support `XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND`
   * the request is silently ignored and VR (opaque) rendering is used.
   *
   * For passthrough to be visible, the renderer background alpha must be 0
   * (transparent) so the runtime can blend real-world content behind the scene.
   *
   * Default value: `false`
   */
  vtkSetMacro(UsePassthrough, bool);
  vtkGetMacro(UsePassthrough, bool);
  vtkBooleanMacro(UsePassthrough, bool);
  ///@}

  ///@{
  /**
   * Set the opacity of virtual geometry that is occluded by real-world surfaces
   * when environment-depth occlusion (`XR_META_environment_depth`) is active.
   *
   * - 1.0 (default): no occlusion — depth composition is skipped entirely.
   * - 0.0 < value < 1.0: partial occlusion — occluded pixels have their alpha
   *   multiplied by this value (post-pass).
   * - 0.0: full occlusion — occluded pixels have their alpha set to 0 (post-pass).
   *
   * This must be set before or during rendering; changes take effect on the
   * next rendered frame.
   *
   * Has no effect if `XR_META_environment_depth` is unavailable or inactive.
   */
  vtkSetClampMacro(OccludedOpacity, float, 0.0f, 1.0f);
  vtkGetMacro(OccludedOpacity, float);
  ///@}

  ///@{
  /**
   * Enable or disable the false-colour environment-depth debug visualisation overlay.
   *
   * When enabled, a semi-transparent red-to-blue colour map of the real-world
   * depth texture is composited on top of each eye's rendered frame.
   * Red = near (0 m), blue = far (~5 m).  Intended for verifying that the
   * depth texture is being received and reprojected correctly.
   *
   * Has no effect if `XR_META_environment_depth` is unavailable or inactive.
   *
   * Default value: `false`
   */
  vtkSetMacro(ShowEnvDepthDebugVisualization, bool);
  vtkGetMacro(ShowEnvDepthDebugVisualization, bool);
  vtkBooleanMacro(ShowEnvDepthDebugVisualization, bool);
  ///@}

  ///@{
  /**
   * When true, the XR_META_environment_depth extension is requested at session
   * creation time even if OccludedOpacity == 1.0 (i.e. depth-occlusion rendering
   * is not active).  Use this when you need raw environment-depth data for
   * purposes other than occlusion (e.g. capturing depth into a volume node).
   *
   * Must be set before the OpenXR session is initialized.
   * Has no effect if the runtime does not support XR_META_environment_depth.
   *
   * Default value: `false`
   */
  vtkSetMacro(CaptureEnvironmentDepth, bool);
  vtkGetMacro(CaptureEnvironmentDepth, bool);
  vtkBooleanMacro(CaptureEnvironmentDepth, bool);
  ///@}

  /**
   * Returns true if passthrough (XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND) is
   * actually active after initialization.
   *
   * If UsePassthrough was requested but the runtime does not support
   * XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND, `Initialize()` falls back to the
   * runtime-preferred mode (typically XR_ENVIRONMENT_BLEND_MODE_OPAQUE) and
   * this method returns false.
   *
   * Only meaningful after Initialize() has been called and returned
   * VRInitialized == true.
   */
  bool IsPassthroughActive() const;

  /**
   * Returns scene observer associated with this window.
   *
   * This function returns `nullptr` if:
   * - The render window has been initialized with `EnableSceneUnderstanding == false` (default)
   * - The runtime does not support scene understanding
   * - The scene observer initialization failed for any other reason
   *
   * \sa SetEnableSceneUnderstanding
   */
  vtkOpenXRSceneObserver* GetSceneObserver();

  /**
   * Structure representing OpenXR instance version
   */
  struct VTKRENDERINGOPENXR_EXPORT InstanceVersion
  {
    std::uint16_t Major{};
    std::uint16_t Minor{};
    std::uint32_t Patch{};
  };

  /**
   * Utility function to get XrInstance runtime version for the given ConnectionStrategy.
   * This function creates a XrInstance, which may have a significant runtime overhead.
   * This function can be called only before the vtkOpenXRRenderWindow has been initialized.
   */
  static InstanceVersion QueryInstanceVersion(vtkOpenXRManagerConnection* cs);

protected:
  vtkOpenXRRenderWindow();
  ~vtkOpenXRRenderWindow() override;

  // Create one framebuffer per view
  bool CreateFramebuffers(uint32_t viewCount = 2) override;

  bool BindTextureToFramebuffer(FramebufferDesc& framebufferDesc);
  void RenderFramebuffer(FramebufferDesc& framebufferDesc) override;

  virtual void RenderOneEye(uint32_t eye);

  vtkNew<vtkMatrix4x4> TempMatrix4x4;

  // Store if a model is active or not here as openxr do not have a concept
  // of active/inactive controller
  std::array<bool, 2> ModelsActiveState = { true, true };

private:
  vtkOpenXRRenderWindow(const vtkOpenXRRenderWindow&) = delete;
  void operator=(const vtkOpenXRRenderWindow&) = delete;

  class vtkInternals;
  std::unique_ptr<vtkInternals> Internal;

  bool UseDepthExtension = false;
  bool EnableSceneUnderstanding = false;
  bool UsePassthrough = false;
  float OccludedOpacity = 1.0f;
  bool ShowEnvDepthDebugVisualization = false;
  bool CaptureEnvironmentDepth = false;

#ifdef XR_META_environment_depth
  // Depth occlusion pre-pass: writes real-world depths to the depth buffer
  // before each eye's scene render so virtual geometry behind real surfaces
  // is naturally occluded. Created on first use, destroyed on ~vtkOpenXRRenderWindow.
  std::unique_ptr<vtkOpenXREnvironmentDepthOcclusionPrePass> EnvDepthPrePass;
#endif
};

VTK_ABI_NAMESPACE_END
#endif
// VTK-HeaderTest-Exclude: vtkOpenXRRenderWindow.h
