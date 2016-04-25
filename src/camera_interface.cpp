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

int setAOI(cameraOptions &cam_options){
  int cameraStatus;

  cam_options.aoiWidth = cam_options.imageWidth / 2;
  cam_options.aoiHeight = cam_options.imageHeight / 2;

  cam_options.aoiPosX = (cam_options.imageWidth / 2) - (cam_options.aoiWidth / 2);
  cam_options.aoiPosY = (cam_options.imageHeight / 2) - (cam_options.aoiHeight / 2);

  IS_RECT areaOfInterest;
  areaOfInterest.s32Width = cam_options.aoiWidth;
  areaOfInterest.s32Height = cam_options.aoiHeight;
  areaOfInterest.s32X = cam_options.aoiPosX;
  areaOfInterest.s32Y = cam_options.aoiPosY;

  cameraStatus = is_AOI(cam_options.camHandle,
                        IS_AOI_IMAGE_SET_AOI,
                        (void*) &areaOfInterest,
                        sizeof(areaOfInterest));

  switch (cameraStatus){
  case IS_SUCCESS:
          cout << "Area of interest set." << endl;
          return SUCCESS;
  case IS_INVALID_PARAMETER:
          cout << "Error setting area of interest. Invalid parameter!" << endl;
          return FAILURE;
  default:
          cout << "Error setting area of interest. Err no. " << cameraStatus << endl;
          return FAILURE;
  } // switch cameraStatus
} // setAOI

int initBuffers(const cameraOptions &cam_options,
                cv::Mat &image,
                cv::Mat &greyImage,
                std::vector<char*> &imgPtrList,
                std::vector<int> &imgIdList){
  int cameraStatus;

  image = cv::Mat(cam_options.aoiHeight,
                  cam_options.aoiWidth,
                  COLOR_DEPTH_CV);
  greyImage = cv::Mat(cam_options.aoiHeight,
                      cam_options.aoiWidth,
                      COLOR_DEPTH_CV_GREY);


  imgPtrList.resize(cam_options.ringBufferSize);
  imgIdList.resize(cam_options.ringBufferSize);


  cameraStatus = is_SetDisplayMode(cam_options.camHandle, IS_SET_DM_DIB);
  switch (cameraStatus){
  case IS_SUCCESS:
          break;
  default:
          cout << "Error setting the display mode to bitmap. Error code: " \
               << cameraStatus << endl;
          return FAILURE;
  }


  for (int i = 0; i < cam_options.ringBufferSize; i++){
    // Allocate memory for the bitmap-image
    cameraStatus = is_AllocImageMem(cam_options.camHandle,
                                    cam_options.aoiWidth,
                                    cam_options.aoiHeight,
                                    COLOR_DEPTH,
                                    &imgPtrList[i],
                                    &imgIdList[i]);

    cameraStatus |= is_AddToSequence(cam_options.camHandle,
                                     imgPtrList[i],
                                     imgIdList[i]);

    if (cameraStatus != IS_SUCCESS){
      std::cout << "Failed to initialize image buffer" << i \
                << ". Error: " << cameraStatus << std::endl;
      return FAILURE;
    }

    std::cout << "Image buffer " << i << " allocated and added to the sequence." << std::endl;
  } // for-loop running through each buffer

  return SUCCESS;
} // initBuffers


void getCaptureStatus (UEYE_CAPTURE_STATUS_INFO capture_status_info){
  cout << "The following errors occured:" << endl;

  if (capture_status_info.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_NO_DEST_MEM])
    cout << "\tNo destination memory for copying." << endl;

  if (capture_status_info.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_CONVERSION_FAILED])
    cout << "\tCurrent image could'nt be processed correctly." << endl;

  if (capture_status_info.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_IMAGE_LOCKED])
    cout << "\tDestination buffers are locked." << endl;

  if (capture_status_info.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_OUT_OF_BUFFERS])
    cout << "\tNo free internal image memory available to the driver." << endl;

  if (capture_status_info.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_DEVICE_NOT_READY])
    cout << "\tCamera no longer available." << endl;

  if (capture_status_info.adwCapStatusCnt_Detail[IS_CAP_STATUS_USB_TRANSFER_FAILED])
    cout << "\tImage wasn't transferred over the USB bus." << endl; // technically shouldn't be possible with the Gige Ueye

  if (capture_status_info.adwCapStatusCnt_Detail[IS_CAP_STATUS_DEV_TIMEOUT])
    cout << "\tThe device reached a timeout." << endl;

  if (capture_status_info.adwCapStatusCnt_Detail[IS_CAP_STATUS_ETH_BUFFER_OVERRUN])
    cout << "\tThe sensor transfers more data than the internal camera memory can accomodate." << endl;

  if (capture_status_info.adwCapStatusCnt_Detail[IS_CAP_STATUS_ETH_MISSED_IMAGES])
    cout << "\tFreerun mode: The GigE uEye camera could neither process nor output an image captured by the sensor.\n\tHardware trigger mode: The GigE uEye camera received a hardware trigger signal which could not be processed because the sensor was still busy." << endl;

} //getCaptureStatus

} // namespace cam
