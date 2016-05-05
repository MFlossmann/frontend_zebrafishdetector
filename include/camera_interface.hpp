#pragma once

#include <iostream>
using std::cout;
using std::endl;

#include <opencv2/opencv.hpp>
#include <ueye.h>

//////////////////// defines
#define CAM_SUCCESS 0
#define CAM_FAILURE -1
#define CAM_WARNING 1

#define CAM_TRIGGER_RISING_EDGE IS_SET_TRIGGER_LO_HI
#define CAM_TRIGGER_SOFTWARE IS_SET_TRIGGER_SOFTWARE

namespace cam{

  enum captureModeEnum{
    HARDWARE_LIVE,
    SOFTWARE_FREEZE
  };

//////////////////// consts
  const int IMAGE_WIDTH = 1280;
  const int IMAGE_HEIGHT = 1024;

  const int COLOR_DEPTH = 8;
  const int COLOR_DEPTH_CV = CV_8UC3;
  const int COLOR_DEPTH_CV_GREY = CV_8UC1;

//////////////////// structs
  struct cameraOptions{
    cameraOptions(){
      imageWidth = IMAGE_WIDTH;
      imageHeight = IMAGE_HEIGHT;
      aoiWidth = imageWidth;
      aoiHeight = imageHeight;
      aoiPosX = 0;
      aoiPosY = 0;

      captureMode = captureModeEnum::HARDWARE_LIVE;

      cameraMatrix = cv::Mat::eye(3,
                                  3,
                                  CV_64F);
      distCoeffs = cv::Mat::zeros(8,
                                  1,
                                  CV_64F);
    }

    HIDS camHandle;

    int imageWidth;
    int imageHeight;
    unsigned int aoiWidth;
    unsigned int aoiHeight;
    int aoiPosX;
    int aoiPosY;
    double framerate;
    double framerateEff;
    double flashTime;
    double exposure;
    unsigned int ringBufferSize;

    bool undistortImage;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    captureModeEnum captureMode;

    cv::Mat image, greyImage;
    std::vector<char*> imgPtrList;
    std::vector<int> imgIdList;
  };

  int initialize(cameraOptions &cam_options);

/*! /brief Sets the trigger mode, the framerate, the exposure time
  and the color depth of the camera.

  /param cam_options includes the camera options

  /return SUCCESS or FAILURE
*/
  int setOptions(cameraOptions &cam_options);

  int setAOI(cameraOptions &cam_options);

  int initBuffers(cameraOptions &cam_options);

  int startVideoCapture(cameraOptions &cam_options);

  void getCaptureStatus(UEYE_CAPTURE_STATUS_INFO capture_status_info);
}
