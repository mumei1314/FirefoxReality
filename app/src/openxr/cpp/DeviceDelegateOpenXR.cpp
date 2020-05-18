/* -*- Mode: C++; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "DeviceDelegateOpenXR.h"
#include "DeviceUtils.h"
#include "ElbowModel.h"
#include "BrowserEGLContext.h"
#include "VRBrowser.h"
#include "VRLayer.h"

#include <android_native_app_glue.h>
#include <EGL/egl.h>
#include "vrb/CameraEye.h"
#include "vrb/Color.h"
#include "vrb/ConcreteClass.h"
#include "vrb/FBO.h"
#include "vrb/GLError.h"
#include "vrb/Matrix.h"
#include "vrb/Quaternion.h"
#include "vrb/RenderContext.h"
#include "vrb/Vector.h"

#include <vector>
#include <array>
#include <cstdlib>
#include <unistd.h>

#include "VRBrowser.h"

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_oculus.h>
#include "OpenXRHelpers.h"

namespace crow {

const vrb::Vector kAverageHeight(0.0f, 1.7f, 0.0f);
// Height used to match Oculus default in WebVR
const vrb::Vector kAverageOculusHeight(0.0f, 1.65f, 0.0f);

struct DeviceDelegateOpenXR::State {

  vrb::RenderContextWeak context;
  android_app* app = nullptr;
  bool applicationEntitled = false;
  bool layersEnabled = true;
  XrInstanceCreateInfoAndroidKHR java;
  XrInstance instance = XR_NULL_HANDLE;
  XrSystemId system = XR_NULL_SYSTEM_ID;
  XrSession session = XR_NULL_HANDLE;
  XrSessionState sessionState = XR_SESSION_STATE_UNKNOWN;
  bool vrReady = false;
  XrGraphicsBindingOpenGLESAndroidKHR graphicsBinding{XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
  XrSystemProperties systemProperties{XR_TYPE_SYSTEM_PROPERTIES};
  XrEventDataBuffer eventBuffer;
  XrViewConfigurationType viewConfig{XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
  std::vector<XrReferenceSpaceType> spaces;
  device::RenderMode renderMode = device::RenderMode::StandAlone;
  vrb::FBOPtr currentFBO;
  vrb::FBOPtr previousFBO;
  vrb::CameraEyePtr cameras[2];
  uint32_t frameIndex = 0;
  FramePrediction framePrediction = FramePrediction::NO_FRAME_AHEAD;
  double prevPredictedDisplayTime = 0;
  double predictedDisplayTime = 0;
  uint32_t discardedFrameIndex = 0;
  int discardCount = 0;
  uint32_t renderWidth = 0;
  uint32_t renderHeight = 0;
  vrb::Color clearColor;
  float near = 0.1f;
  float far = 100.f;
  bool hasEventFocus = true;
  crow::ElbowModelPtr elbow;
  ControllerDelegatePtr controller;
  ImmersiveDisplayPtr immersiveDisplay;
  int reorientCount = -1;
  vrb::Matrix reorientMatrix = vrb::Matrix::Identity();
  device::CPULevel minCPULevel = device::CPULevel::Normal;
  device::DeviceType deviceType = device::UnknownType;

  void UpdatePerspective() {
  }

  void Initialize() {
    elbow = ElbowModel::Create();
    vrb::RenderContextPtr localContext = context.lock();

    // Adhoc loader required for Oculus
    XrLoaderInitializeInfoAndroidOCULUS loaderData;
    memset(&loaderData, 0, sizeof(loaderData));
    loaderData.type = XR_TYPE_LOADER_INITIALIZE_INFO_ANDROID_OCULUS;
    loaderData.next = nullptr;
    loaderData.applicationVM = app->activity->vm;
    JNIEnv* env = nullptr;
    (*app->activity->vm).AttachCurrentThread(&env, NULL);
    loaderData.applicationActivity = env->NewGlobalRef(app->activity->clazz);
    xrInitializeLoaderOCULUS(&loaderData);

    // Initialize the XrInstance
    std::array<const char *, 2> extensions = {
       XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,
       XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME
    };
    java = {XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
    java.applicationVM = app->activity->vm;
    java.applicationActivity = env->NewGlobalRef(app->activity->clazz);

    XrInstanceCreateInfo createInfo{XR_TYPE_INSTANCE_CREATE_INFO};
    createInfo.next = (XrBaseInStructure*)&java;
    createInfo.enabledExtensionCount = (uint32_t)extensions.size();
    createInfo.enabledExtensionNames = extensions.data();
    strcpy(createInfo.applicationInfo.applicationName, "Firefox Reality");
    createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

    CHECK_XRCMD(xrCreateInstance(&createInfo, &instance));
    CHECK_MSG(instance != XR_NULL_HANDLE, "Failed to create XRInstance");

    XrInstanceProperties instanceProperties{XR_TYPE_INSTANCE_PROPERTIES};
    CHECK_XRCMD(xrGetInstanceProperties(instance, &instanceProperties));
    VRB_LOG("OpenXR Instance Created: RuntimeName=%s RuntimeVersion=%s", instanceProperties.runtimeName,
            GetXrVersionString(instanceProperties.runtimeVersion).c_str());


    // Initialize System
    XrSystemGetInfo systemInfo{XR_TYPE_SYSTEM_GET_INFO};
    systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    CHECK_XRCMD(xrGetSystem(instance, &systemInfo, &system));
    CHECK_MSG(system != XR_NULL_SYSTEM_ID, "Failed to initialize XRSystem");

    // Retrieve system info
    CHECK_XRCMD(xrGetSystemProperties(instance, system, &systemProperties))
    VRB_LOG("OpenXR system name: %s", systemProperties.systemName);

    layersEnabled = VRBrowser::AreLayersEnabled();
  }

  // xrGet*GraphicsRequirementsKHR check must be called prior to xrCreateSession
  // xrCreateSession fails if we don't call it.
  void CheckGraphicsRequirements() {
    // Extension function must be loaded by name
    PFN_xrGetOpenGLESGraphicsRequirementsKHR pfnGetOpenGLESGraphicsRequirementsKHR = nullptr;
    CHECK_XRCMD(xrGetInstanceProcAddr(instance, "xrGetOpenGLESGraphicsRequirementsKHR",
                                      reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetOpenGLESGraphicsRequirementsKHR)));

    XrGraphicsRequirementsOpenGLESKHR graphicsRequirements{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
    CHECK_XRCMD(pfnGetOpenGLESGraphicsRequirementsKHR(instance, system, &graphicsRequirements));

    GLint major = 0;
    GLint minor = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);

    const XrVersion desiredApiVersion = XR_MAKE_VERSION(major, minor, 0);
    if (graphicsRequirements.minApiVersionSupported > desiredApiVersion) {
      THROW("Runtime does not support desired Graphics API and/or version");
    }
  }

  void UpdateSpaces() {
    CHECK(session != XR_NULL_HANDLE);

    uint32_t spaceCount = 0;
    CHECK_XRCMD(xrEnumerateReferenceSpaces(session, 0, &spaceCount, nullptr));
    std::vector<XrReferenceSpaceType> query(spaceCount);
    CHECK_XRCMD(xrEnumerateReferenceSpaces(session, spaceCount, &spaceCount, query.data()));
    spaces = query;

    VRB_DEBUG("OpenXR Available reference spaces: %d", spaceCount);
    for (XrReferenceSpaceType space : spaces) {
      VRB_DEBUG("  OpenXR Space Name: %s", to_string(space));
    }
  }

  void UpdateTrackingMode() {

  }

  void UpdateClockLevels() {

  }

  void UpdateDisplayRefreshRate() {

  }

  void UpdateBoundary() {
  }

  bool Is6DOF() const {
    return systemProperties.trackingProperties.positionTracking != 0;
  }

  const XrEventDataBaseHeader* PollEvent() {
    if (!instance) {
      return nullptr;
    }
    XrEventDataBaseHeader* baseHeader = reinterpret_cast<XrEventDataBaseHeader*>(&eventBuffer);
    *baseHeader = {XR_TYPE_EVENT_DATA_BUFFER};
    const XrResult xr = xrPollEvent(instance, &eventBuffer);
    if (xr == XR_SUCCESS) {
      return baseHeader;
    }

    CHECK_MSG(xr == XR_EVENT_UNAVAILABLE, "Expected XR_EVENT_UNAVAILABLE result")
    return nullptr;
  }

  void HandleSessionEvent(const XrEventDataSessionStateChanged& event) {
    VRB_DEBUG("OpenXR XrEventDataSessionStateChanged: state %s->%s session=%p time=%ld",
        to_string(sessionState), to_string(event.state), event.session, event.time);
    sessionState = event.state;

    CHECK(session == event.session);

    switch (sessionState) {
      case XR_SESSION_STATE_READY: {
        XrSessionBeginInfo sessionBeginInfo{XR_TYPE_SESSION_BEGIN_INFO};
        sessionBeginInfo.primaryViewConfigurationType = viewConfig;
        CHECK_XRCMD(xrBeginSession(session, &sessionBeginInfo));
        vrReady = true;
        break;
      }
      case XR_SESSION_STATE_STOPPING: {
        vrReady = false;
        CHECK_XRCMD(xrEndSession(session))
        break;
      }
      case XR_SESSION_STATE_EXITING: {
        vrReady = false;
        break;
      }
      case XR_SESSION_STATE_LOSS_PENDING: {
        vrReady = false;
        break;
      }
      default:
        break;
    }
  }


  void Shutdown() {
    // Shutdown Oculus mobile SDK
    if (instance) {
      CHECK_XRCMD(xrDestroyInstance(instance));
      instance = XR_NULL_HANDLE;
    }

    // TODO: Check if activity globarRef needs to be released
  }
};

DeviceDelegateOpenXRPtr
DeviceDelegateOpenXR::Create(vrb::RenderContextPtr& aContext, android_app *aApp) {
  DeviceDelegateOpenXRPtr result = std::make_shared<vrb::ConcreteClass<DeviceDelegateOpenXR, DeviceDelegateOpenXR::State> >();
  result->m.context = aContext;
  result->m.app = aApp;
  result->m.Initialize();
  return result;
}

device::DeviceType
DeviceDelegateOpenXR::GetDeviceType() {
  return m.deviceType;
}

void
DeviceDelegateOpenXR::SetRenderMode(const device::RenderMode aMode) {
  if (aMode == m.renderMode) {
    return;
  }
  /*m.renderMode = aMode;
  m.SetRenderSize(aMode);
  vrb::RenderContextPtr render = m.context.lock();
  for (int i = 0; i < VRAPI_EYE_COUNT; ++i) {
    m.eyeSwapChains[i]->Init(render, m.renderMode, m.renderWidth, m.renderHeight);
  }*/

  m.UpdateTrackingMode();
  m.UpdateDisplayRefreshRate();
  m.UpdateClockLevels();

  // Reset reorient when exiting or entering immersive
  m.reorientMatrix = vrb::Matrix::Identity();
}

device::RenderMode
DeviceDelegateOpenXR::GetRenderMode() {
  return m.renderMode;
}

void
DeviceDelegateOpenXR::RegisterImmersiveDisplay(ImmersiveDisplayPtr aDisplay) {
  m.immersiveDisplay = std::move(aDisplay);

  if (!m.immersiveDisplay) {
    return;
  }
  if (m.Is6DOF()) {
    m.immersiveDisplay->SetDeviceName("Oculus Quest");
  } else {
    m.immersiveDisplay->SetDeviceName("Oculus Go");
  }
  uint32_t width = 500, height = 500;
  m.immersiveDisplay->SetEyeResolution(width, height);
  m.immersiveDisplay->SetSittingToStandingTransform(vrb::Matrix::Translation(kAverageOculusHeight));
  m.UpdateBoundary();
  m.UpdatePerspective();

  m.immersiveDisplay->CompleteEnumeration();
}

void
DeviceDelegateOpenXR::SetImmersiveSize(const uint32_t aEyeWidth, const uint32_t aEyeHeight) {

}

vrb::CameraPtr
DeviceDelegateOpenXR::GetCamera(const device::Eye aWhich) {
  const int32_t index = device::EyeIndex(aWhich);
  if (index < 0) { return nullptr; }
  return m.cameras[index];
}

const vrb::Matrix&
DeviceDelegateOpenXR::GetHeadTransform() const {
  return m.cameras[0]->GetHeadTransform();
}

const vrb::Matrix&
DeviceDelegateOpenXR::GetReorientTransform() const {
  return m.reorientMatrix;
}

void
DeviceDelegateOpenXR::SetReorientTransform(const vrb::Matrix& aMatrix) {
  m.reorientMatrix = aMatrix;
}

void
DeviceDelegateOpenXR::SetClearColor(const vrb::Color& aColor) {
  m.clearColor = aColor;
}

void
DeviceDelegateOpenXR::SetClipPlanes(const float aNear, const float aFar) {
  m.near = aNear;
  m.far = aFar;
  m.UpdatePerspective();
}

void
DeviceDelegateOpenXR::SetControllerDelegate(ControllerDelegatePtr& aController) {
  m.controller = aController;
}

void
DeviceDelegateOpenXR::ReleaseControllerDelegate() {
  m.controller = nullptr;
}

int32_t
DeviceDelegateOpenXR::GetControllerModelCount() const {
  return 0;
}

const std::string
DeviceDelegateOpenXR::GetControllerModelName(const int32_t aModelIndex) const {
  static std::string name = "";

  if (m.Is6DOF()) {
    switch (aModelIndex) {
      case 0:
        name = "vr_controller_oculusquest_left.obj";
        break;
      case 1:
        name = "vr_controller_oculusquest_right.obj";
        break;
      default:
        VRB_WARN("GetControllerModelName() failed.");
        name = "";
        break;
    }
  } else {
    name = "vr_controller_oculusgo.obj";
  }

  return name;
}


void
DeviceDelegateOpenXR::SetCPULevel(const device::CPULevel aLevel) {
  m.minCPULevel = aLevel;
  m.UpdateClockLevels();
};


void
DeviceDelegateOpenXR::ProcessEvents() {
  while (const XrEventDataBaseHeader* ev = m.PollEvent()) {
    switch (ev->type) {
      case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
        const auto& event = *reinterpret_cast<const XrEventDataSessionStateChanged*>(ev);
        m.HandleSessionEvent(event);
        break;
      }
      case XR_TYPE_EVENT_DATA_EVENTS_LOST: {
        const auto& event = *reinterpret_cast<const XrEventDataEventsLost*>(ev);
        VRB_WARN("OpenXR %d events lost", event.lostEventCount);
        break;
      }
      case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
        // Receiving the XrEventDataInstanceLossPending event structure indicates that the application
        // is about to lose the indicated XrInstance at the indicated lossTime in the future.
        const auto& event = *reinterpret_cast<const XrEventDataInstanceLossPending*>(ev);
        VRB_WARN("OpenXR XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING by %ld", event.lossTime);
        m.vrReady = false;
        return;
      }
      case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
      default: {
        VRB_DEBUG("OpenXR ignoring event type %d", ev->type);
        break;
      }
    }
  }
}

bool
DeviceDelegateOpenXR::SupportsFramePrediction(FramePrediction aPrediction) const {
  return false;
}

void
DeviceDelegateOpenXR::StartFrame(const FramePrediction aPrediction) {
  if (!m.vrReady) {
    VRB_ERROR("OpenXR StartFrame called while not in VR mode");
    return;
  }
  VRB_GL_CHECK(glClearColor(m.clearColor.Red(), m.clearColor.Green(), m.clearColor.Blue(), m.clearColor.Alpha()));
}

void
DeviceDelegateOpenXR::BindEye(const device::Eye aWhich) {
  if (!m.vrReady) {
    VRB_ERROR("OpenXR BindEye called while not in VR mode");
    return;
  }

  int32_t index = device::EyeIndex(aWhich);
  if (index < 0) {
    VRB_LOG("No eye found");
    return;
  }

  if (m.currentFBO) {
    m.currentFBO->Unbind();
  }

  /*const auto &swapChain = m.eyeSwapChains[index];
  int swapChainIndex = m.frameIndex % swapChain->swapChainLength;
  m.currentFBO = swapChain->fbos[swapChainIndex];*/

  if (m.currentFBO) {
    m.currentFBO->Bind();
    VRB_GL_CHECK(glViewport(0, 0, m.renderWidth, m.renderHeight));
    VRB_GL_CHECK(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
  } else {
    VRB_LOG("No Swap chain FBO found");
  }

  /*for (const OculusLayerPtr& layer: m.uiLayers) {
    layer->SetCurrentEye(aWhich);
  }*/
}

void
DeviceDelegateOpenXR::EndFrame(const FrameEndMode aEndMode) {
  if (!m.vrReady) {
    VRB_ERROR("OpenXR EndFrame called while not in VR mode");
    return;
  }
  if (m.currentFBO) {
    m.currentFBO->Unbind();
    m.currentFBO.reset();
  }
}

VRLayerQuadPtr
DeviceDelegateOpenXR::CreateLayerQuad(int32_t aWidth, int32_t aHeight,
                                        VRLayerSurface::SurfaceType aSurfaceType) {
  return nullptr;
}

VRLayerQuadPtr
DeviceDelegateOpenXR::CreateLayerQuad(const VRLayerSurfacePtr& aMoveLayer) {
  return nullptr;
}

VRLayerCylinderPtr
DeviceDelegateOpenXR::CreateLayerCylinder(int32_t aWidth, int32_t aHeight,
                                            VRLayerSurface::SurfaceType aSurfaceType) {
  return nullptr;
}

VRLayerCylinderPtr
DeviceDelegateOpenXR::CreateLayerCylinder(const VRLayerSurfacePtr& aMoveLayer) {
  return nullptr;
}


VRLayerCubePtr
DeviceDelegateOpenXR::CreateLayerCube(int32_t aWidth, int32_t aHeight, GLint aInternalFormat) {
  return nullptr;
}

VRLayerEquirectPtr
DeviceDelegateOpenXR::CreateLayerEquirect(const VRLayerPtr &aSource) {
  return nullptr;
}

void
DeviceDelegateOpenXR::DeleteLayer(const VRLayerPtr& aLayer) {
}

void
DeviceDelegateOpenXR::EnterVR(const crow::BrowserEGLContext& aEGLContext) {
  // Reset reorientation after Enter VR
  m.reorientMatrix = vrb::Matrix::Identity();

  if (m.session != XR_NULL_HANDLE && m.graphicsBinding.context == aEGLContext.Context()) {
    // Session already created and valid.
    return;
  }

  CHECK(m.instance != XR_NULL_HANDLE && m.system != XR_NULL_SYSTEM_ID);
  m.CheckGraphicsRequirements();

  m.graphicsBinding.context = aEGLContext.Context();
  m.graphicsBinding.display = aEGLContext.Display();
  m.graphicsBinding.config = (EGLConfig)0;

  XrSessionCreateInfo createInfo{XR_TYPE_SESSION_CREATE_INFO};
  createInfo.next = reinterpret_cast<const XrBaseInStructure*>(&m.graphicsBinding);
  createInfo.systemId = m.system;
  CHECK_XRCMD(xrCreateSession(m.instance, &createInfo, &m.session));
  CHECK(m.session != XR_NULL_HANDLE);
  VRB_LOG("OpenXR session created succesfully");

  m.UpdateSpaces();
  ProcessEvents();
}


void
DeviceDelegateOpenXR::LeaveVR() {
  m.currentFBO = nullptr;
  m.previousFBO = nullptr;
  ProcessEvents();
  m.vrReady = false;
}

void
DeviceDelegateOpenXR::OnDestroy() {
}

bool
DeviceDelegateOpenXR::IsInVRMode() const {
  return m.vrReady;
}

bool
DeviceDelegateOpenXR::ExitApp() {
  return true;
}

DeviceDelegateOpenXR::DeviceDelegateOpenXR(State &aState) : m(aState) {}

DeviceDelegateOpenXR::~DeviceDelegateOpenXR() { m.Shutdown(); }

} // namespace crow
