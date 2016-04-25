#include "camera_interface.hpp"

namespace cam{

int initialize(cameraOptions &cam_options){
  cam_options.camHandle = 0; // select the first available camera.
  int cameraStatus = is_InitCamera(&cam_options.camHandle, NULL);

  switch (cameraStatus){
  case IS_SUCCESS:
    std::cout << "Camera " << cam_options.camHandle << " initialized!" << std::endl;
    return SUCCESS;

  case IS_CANT_OPEN_DEVICE:
    cout << "Error initializing, can't access the camera. Is it connected?\n Are you sure, it has power?" << endl;
    return FAILURE;

  default:
    std::cout << "Error initializing camera. Err no. " << cameraStatus << std::endl;
    return FAILURE;
  } // switch cameraStatus

} // initialize


int setOptions(cameraOptions &cam_options){
  int cameraStatus;

  // Trigger mode
  cameraStatus = is_SetExternalTrigger(cam_options.camHandle,
                                        IS_SET_TRIGGER_SOFTWARE);

  switch (cameraStatus){
  case IS_SUCCESS:
    break;
  default:
    cout << "Error setting the trigger! Err no. " << cameraStatus << endl;
    return FAILURE;
  } // switch cameraStatus


  // Framerate
  cameraStatus = is_SetFrameRate(cam_options.camHandle,
                                  cam_options.framerate,
                                  &cam_options.framerateEff);

  switch (cameraStatus){
  case IS_SUCCESS:
    cout << "Framerate was set to " << cam_options.framerate		\
   << "fps, the effective framerate is " << cam_options.framerateEff \
   << "fps." << endl;
    break;
  default:
    cout << "Error setting framerate. Err no. " << cameraStatus << endl;
    return FAILURE;
  } // switch cameraStatus


  // Exposure time set to specific value
  cameraStatus = is_Exposure(cam_options.camHandle,
                             IS_EXPOSURE_CMD_SET_EXPOSURE,
                             &cam_options.exposure,
                             sizeof(cam_options.exposure));
  switch (cameraStatus){
  case IS_SUCCESS:
    cout << "Exposure time set to " << cam_options.exposure	\
         << "ms." << endl;
    break;
  default:
    cout << "Error setting exposure time. Err no. " << cameraStatus << endl;
    return FAILURE;
  } // switch cameraStatus

  // Color depth
  cameraStatus = is_SetColorMode(cam_options.camHandle,
                                 IS_CM_MONO8);

  switch (cameraStatus){
  case IS_SUCCESS:
    break;
  default:
    cout << "Error setting the color mode! Err no. " << cameraStatus << endl;
    return FAILURE;
  } // switch cameraStatus

  return SUCCESS;
} // setOptions

} // namespace cam
