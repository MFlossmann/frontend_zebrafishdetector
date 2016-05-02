#include <iostream>

#include "camera_interface.hpp"
#include "xyPlotter.hpp"

using std::cout;
using std::cerr;
using std::endl;

//////////////////// Consts
const xyBaudRate BAUD_RATE = xyBaudRate::BAUD_57600;
const int KEY_ESCAPE = 27;

const double FRAMERATE = 10.;
const double FLASHTIME = 0.2;
const double EXPOSURE = 4;

//////////////////// MAIN FUNCTION ////////////////////
int main(int arc, char** arv){
//////////////////// Init serial communication
  xyPlotter xy_plotter;

  if (xy_plotter.connect("/dev/ttyUSB0",
                         BAUD_RATE != XY_SUCCESS)){
    cerr << "Error initiating serial communication! Terminating..." << endl;
    return -1;
  }
  else
    cout << "Serial communication initiated..." << endl;

//////////////////// Connect to the camera
  cam::cameraOptions cam_options;
  cam_options.framerate = FRAMERATE;
  cam_options.framerateEff = FRAMERATE;
  cam_options.flashTime = FLASHTIME;
  cam_options.exposure = EXPOSURE;
  cam_options.ringBufferSize = 16;
  cam_options.undistortImage = false;

  int status = 0;
  status |= cam::initialize(cam_options);
  status |= cam::setOptions(cam_options);
  status |= cam::initBuffers(cam_options);

  if (status != CAM_SUCCESS)
    return -1;

  int captureStatus;
  do
  {
    is_FreezeVideo(cam_options_.camHandle,
                                   IS_WAIT);

    captureStatus = is_WaitForNextImage(cam_options_,
                                        0,
                                        &currentImgPtr,
                                        &currentImgIndex);

    switch (captureStatus) {
    case IS_SUCCESS:

      break;
    case IS_TIMED_OUD:
      cout << "Timeout..." << endl;
    case IS_CAPTURE_STATUS: { // specific error
      UEYE_CAPTURE_STATUS_INFO CaptureStatusInfo;
      is_CaptureStatus(cam_options_.camHandle,
                       IS_CAPTURE_STATUS_INFO_CMD_GET,
                       (void*)&CaptureStatusInfo,
                       sizeof(CaptureStatusInfo));
      cam::getCaptureStatus(CaptureStatusInfo);
      break;
    }
    default:
      cout << "Error capturing image! Error code: " << captureStatus << endl;
      break;
    } // switch captureStatus;

// copy the image to a opencv-handleable format.
    cam_options_.greyImage.data = static_cast<uchar*> currentImgPtr;

    cam_options_.image = cam_options_.greyImage.clone();
    cv::cvtColor(cam_options_.image,
                 cam_options_.image,
                 CV_GRAY2RGB);

// Release the image buffer
    captureStatus |= is_UnlockSeqBuf(cam_options_.camHandle,
                                     currentImgIndex,
                                     currentPtr);

    cv::namedWindow("Image View",
                    CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    cv::imshow("Image View",
               cam_options_.image);

  } while (cv::waitKey(10) != KEY_ESCAPE); // image capture loop.
}
