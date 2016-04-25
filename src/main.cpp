/*
Modeled after this homepage:
https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual/index.html?is_initimagequeue.html

current goal: video mode with freerun mode

 */

#include <iostream>
#include <stdio.h>
#include <string>
#include <chrono>

//#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <ueye.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace po = boost::program_options;
using std::cout;
using std::endl;

//////////////////// typedefs
typedef std::chrono::high_resolution_clock Time;

//////////////////// consts
const std::string CONFIG_FILE_DEFAULT = "../cfg/front_end.cfg";

const int IMAGE_WIDTH = 1280;
const int IMAGE_HEIGHT = 1024;
const int COLOR_DEPTH = 8;
const int COLOR_DEPTH_CV = CV_8UC3;
const int COLOR_DEPTH_CV_GREY = CV_8UC1;
const double DEFAULT_FRAMERATE = 30.0;
const double DEFAULT_EXPOSURE_MS = 4;
const int IMAGE_TIMEOUT = 500;
const int TIMEOUT_COUNTER_MAX = 10;

const int RINGBUFFER_SIZE_DEFAULT = 10;
const std::string WINDOW_NAME = "Display Image (ESC to close)";

const cv::Scalar YELLOW = cv::Scalar(0,0xFF,0xFF);
const cv::Scalar RED = cv::Scalar(0,0,0xFF);

const int KEY_ESCAPE = 27;

//////////////////// structs
struct cameraOptions{
  cameraOptions(){
    cameraMatrix = cv::Mat::eye(3,
                            3,
                            CV_64F);
    distCoeffs = cv::Mat::zeros(8,
                            1,
                            CV_64F);
  }

  double framerate;
  double framerateEff;
  double exposure;
  unsigned int ringBufferSize;

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
};

//////////////////// globals
cameraOptions cam_options_;

//////////////////// functions
std::string getBinPath() {
  char buff[PATH_MAX];
  ssize_t len = ::readlink("/proc/self/exe",
                           buff,
                           sizeof(buff) - 2); // leave one index space for the \0 character

  while (--len > 0 && buff[len] != '/'); // remove the filename

  buff[++len] = '\0';
  return std::string(buff);
}

int parseOptions(int argc, char* argv[]){
  std::string config_file_path, binary_path;

// get binary path
  binary_path = getBinPath();
  if (binary_path.empty()){
    cout << "Error collecting binary path! Terminating" << endl;
    return -1;
  }

  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "Produce help message")
    ("config,c", po::value<std::string>(&config_file_path)->default_value(binary_path + CONFIG_FILE_DEFAULT), "Different configuration file.")
    ;

  // Options that are allowed both on command line and in the config file
  po::options_description config("Configuration");
  config.add_options()
    ("framerate,f", po::value<double>(&cam_options_.framerate)->default_value(DEFAULT_FRAMERATE), "set framerate (fps)")
    ("exposure,e", po::value<double>(&cam_options_.exposure)->default_value(DEFAULT_EXPOSURE_MS), "set exposure time (ms)")
    ("buffer_size,b", po::value<unsigned int>(&cam_options_.ringBufferSize)->default_value(RINGBUFFER_SIZE_DEFAULT), "size of image ringbuffer")
    ;

  po::options_description cmdline_options;
  cmdline_options.add(generic).add(config);

  po::options_description config_file_options;
  config_file_options.add(config);

  po::variables_map variable_map;
  store(po::command_line_parser(argc, argv).
        options(cmdline_options).run(), variable_map);

  notify(variable_map);

  std::ifstream ifs(config_file_path.c_str(),
                    std::ifstream::in);

  {
    boost::filesystem::path tmp(config_file_path);

//FIXXXME: check if the file exists or not!
    if (!ifs.is_open()){
    cout << "can't open config file: " << config_file_path << endl;
    return 2;
  }
  else
    {
       store(parse_config_file(ifs, config_file_options), variable_map);
       notify(variable_map);
    }
  }

  if (variable_map.count("help")) {
    cout << cmdline_options << "\n";
    return 1;
  }

  return 0;
}



void parseCaptureStatus(UEYE_CAPTURE_STATUS_INFO CaptureStatusInfo){
  cout << "The following errors occured:" << endl;

  if (CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_NO_DEST_MEM])
    cout << "\tNo destination memory for copying." << endl;

  if (CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_CONVERSION_FAILED])
    cout << "\tCurrent image could'nt be processed correctly." << endl;

  if (CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_IMAGE_LOCKED])
    cout << "\tDestination buffers are locked." << endl;

  if (CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_OUT_OF_BUFFERS])
    cout << "\tNo free internal image memory available to the driver." << endl;

  if (CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_DEVICE_NOT_READY])
    cout << "\tCamera no longer available." << endl;

  if (CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_USB_TRANSFER_FAILED])
    cout << "\tImage wasn't transferred over the USB bus." << endl; // technically shouldn't be possible with the Gige Ueye

  if (CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_DEV_TIMEOUT])
    cout << "\tThe device reached a timeout." << endl;

  if (CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_ETH_BUFFER_OVERRUN])
    cout << "\tThe sensor transfers more data than the internal camera memory can accomodate." << endl;

  if (CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_ETH_MISSED_IMAGES])
    cout << "\tFreerun mode: The GigE uEye camera could neither process nor output an image captured by the sensor.\n\tHardware trigger mode: The GigE uEye camera received a hardware trigger signal which could not be processed because the sensor was still busy." << endl;
}

void readCalibrationParameters(cv::Mat &cameraMatrix,
                               cv::Mat &distCoeffs){
  // FIXXXXXME: make this dynamic!!
  //cv::FileStorage fs("../fg/calibration.yml", cv::FileStorage::READ);
  cv::FileStorage fs("/home/prunebutt/Documents/uni/masterthesis/cfg/camera_calibration.yml",
                     cv::FileStorage::READ);

  if (!fs.isOpened()){
    cout << "Couldn't open a recalibration file. Image will be distorted." << endl;

    return;
  }

  fs["Camera_Matrix"] >> cameraMatrix;
  fs["Distortion_Coefficients"] >> distCoeffs;

  return;
}

int main(int argc, char* argv[]){
// return if --help option was called
  if (parseOptions(argc, argv))
    return 1;

  readCalibrationParameters(cam_options_.cameraMatrix,
                            cam_options_.distCoeffs);

  //////////////////// Init the camera
  HIDS camHandle = 0; // select the first available camera.
  int cameraStatus = is_InitCamera(&camHandle, NULL);
  switch (cameraStatus){
  case IS_SUCCESS:
    std::cout << "Camera " << camHandle << " initialized!" << std::endl;
    break;
  case IS_CANT_OPEN_DEVICE:
    cout << "Error initializing, can't access the camera. Is it connected?\n Are you sure, it has power?" << endl;
    return -1;
  default:
    std::cout << "Error initializing camera. Err no. " << cameraStatus << std::endl;
    return -1;
  } // switch cameraStatus


  //////////////////// Set camera options
  // Trigger mode
  is_SetExternalTrigger(camHandle,
                        IS_SET_TRIGGER_SOFTWARE); // Set to SW trigger for free video capture

  // Framerate
  cameraStatus = is_SetFrameRate(camHandle,
                                 cam_options_.framerate,
                                 &cam_options_.framerateEff);
  switch (cameraStatus){
  case IS_SUCCESS:
    cout << "Framerate was set to " << cam_options_.framerate		\
   << "fps, the effective framerate is " << cam_options_.framerateEff \
   << "fps." << endl;
    break;
  default:
    cout << "Error setting framerate. Err no. " << cameraStatus << endl;
  }

  // Exposure time set to specific value
  cameraStatus = is_Exposure(camHandle,
                             IS_EXPOSURE_CMD_SET_EXPOSURE,
                             &cam_options_.exposure,
                             sizeof(cam_options_.exposure));
  switch (cameraStatus){
  case IS_SUCCESS:
    cout << "Exposure time set to " << cam_options_.exposure	\
         << "ms." << endl;
    break;
  default:
    cout << "Error setting exposure time. Err no. " << cameraStatus << endl;
  }

  // Color depth
  is_SetColorMode(camHandle,
                  IS_CM_MONO8);


  //////////////////// Init the image buffers
  cv::Mat image(IMAGE_HEIGHT,IMAGE_WIDTH, COLOR_DEPTH_CV);
  cv::Mat greyImage(IMAGE_HEIGHT,IMAGE_WIDTH, COLOR_DEPTH_CV_GREY);

  std::vector<char*> imgPtrList;
  std::vector<int> imgIdList;

  imgPtrList.resize(cam_options_.ringBufferSize);
  imgIdList.resize(cam_options_.ringBufferSize);


  is_SetDisplayMode(camHandle, IS_SET_DM_DIB); // TODO: Where to put this?

  for (int i = 0; i < cam_options_.ringBufferSize; i++){
    // Allocate memory for the bitmap-image
    int status = is_AllocImageMem(camHandle,
                                  IMAGE_WIDTH,
                                  IMAGE_HEIGHT,
                                  COLOR_DEPTH,
                                  &imgPtrList[i],
                                  &imgIdList[i]);

    status |= is_AddToSequence(camHandle,
                               imgPtrList[i],
                               imgIdList[i]);

    if (status != IS_SUCCESS){
      std::cout << "Failed to initialize image buffer" << i \
                << ". Error: " << status << std::endl;
      return -1;
    }

    std::cout << "Image buffer " << i << " allocated and added to the sequence." << std::endl;
  }

  // Activate the image queue TODO: necessary?
  is_InitImageQueue(camHandle,
                    0); // 0 is the only nMode supported


// enable the event that a new image is available
  is_EnableEvent(camHandle,
                 IS_SET_EVENT_FRAME);

  // enable video capturing
  is_CaptureVideo(camHandle, IS_WAIT);

//////////////////// Image capture loop ////////////////////
  std::chrono::duration<float> frameDelta;
  auto currentTime = Time::now(),
    previousTime=Time::now();

  do {

    char* currentImgPtr = 0;
    int currentImgIndex = -1;

//////////////////// Capture the image
    int timeoutCounter = 0;
    int captureStatus;
    do{
      captureStatus = is_WaitForNextImage(camHandle,
                                          IMAGE_TIMEOUT,
                                          &currentImgPtr,
                                          &currentImgIndex);

      switch(captureStatus){
      case IS_SUCCESS:
#ifdef _VERBOSE_MODE_
        cout << "Image captured!" << endl;
#endif
      case IS_TIMED_OUT:
        break;
      case IS_CAPTURE_STATUS:   // Specific error
        UEYE_CAPTURE_STATUS_INFO CaptureStatusInfo;
        is_CaptureStatus(camHandle,
                         IS_CAPTURE_STATUS_INFO_CMD_GET,
                         (void*)&CaptureStatusInfo,
                         sizeof(CaptureStatusInfo));
        parseCaptureStatus(CaptureStatusInfo);
        // is_CaptureStatus(camHandle,
        //                  IS_CAPTURE_STATUS_INFO_CMD_RESET,
        //                  0,
        //                  0); // reset the camera status
        break;
      default:
        cout << "Error capturing image! Error code: " << captureStatus << endl;
        break;
      }
    }
    while (captureStatus == IS_TIMED_OUT ||
           ++timeoutCounter >= TIMEOUT_COUNTER_MAX);

    if (timeoutCounter >= TIMEOUT_COUNTER_MAX)
      cout << "Image timeout!" << endl;

//////////////////// Copy the image buffer
    if (captureStatus == IS_SUCCESS){ // Only copy if an image was received

#ifdef _GREYSCALE_IMAGE_ // deepcopy the greyscale matrix, shallowcopy the grey Image
      // copy the image memory to the openCV memory
      captureStatus = is_CopyImageMem(camHandle,
                                      currentImgPtr,
                                      currentImgIndex,
                                      (char*)greyImage.data);
      image = greyImage; // shallow copy
#else // converts image to color (previously:only copy the pointer)
      greyImage.data = (uchar*) currentImgPtr;

      image = greyImage.clone(); // deep copy
      cv::cvtColor(image,
                   image,
                   CV_GRAY2RGB);
#endif // _GREYSCALE_IMAGE_

      captureStatus |= is_UnlockSeqBuf(camHandle,
                                       currentImgIndex,
                                       currentImgPtr); // Release the image buffer again.

      if(captureStatus != IS_SUCCESS)
        cout << "Error copying buffer!" << endl;
#ifdef _VERBOSE_MODE_
      else {
        cout << "Buffer " << currentImgIndex << " copied and released." << endl;
      }
#endif // _VERBOSE_MODE_

//////////////////// Undistort the image
      cv::Mat tmp = image.clone();
        cv::undistort(tmp,
                      image,
                      cam_options_.cameraMatrix,
                      cam_options_.distCoeffs);

    } // captureStatus == IS_SUCCESS

//////////////////// No new image received
    else {
      cout << "No image received, copying the old one." << endl;

      cv::putText(image,
                  "Image out of date!",
                  cv::Point(image.cols*.1,
                            image.rows*.94),
                  CV_FONT_HERSHEY_PLAIN,
                  5.0,
                  RED,
                  3);
    }

// FIXXME
//////////////////// FPS display
    currentTime = Time::now();
    frameDelta = currentTime - previousTime;
    previousTime = currentTime;
    float fps = 1/frameDelta.count();

    std::stringstream fpsDisplay;
    fpsDisplay << "FPS: " << fps;

    cv::putText(image,
                fpsDisplay.str().c_str(), // stringstream -> string -> c-style string
                cv::Point(150,50),
                CV_FONT_HERSHEY_PLAIN,
                2.0,
                YELLOW);
// END FIXXME

//////////////////// Display the image
    cv::namedWindow(WINDOW_NAME,
                    CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
    cv::imshow(WINDOW_NAME,
               image);

  }while(cv::waitKey(10) != KEY_ESCAPE);

//////////////////// Cleanup and exit

  //TODO: Return variables
  is_StopLiveVideo(camHandle,
                   IS_FORCE_VIDEO_STOP);
  is_ClearSequence(camHandle);

  // Free image buffers
  for (int i=0; i < cam_options_.ringBufferSize; i++){
    is_FreeImageMem(camHandle, imgPtrList[i], imgIdList[i]);
  }
  imgPtrList.clear();
  imgIdList.clear();

  // Close events, buffers, queues and camera
  is_DisableEvent(camHandle,IS_SET_EVENT_FRAME);
  is_ExitImageQueue(camHandle);
  is_ClearSequence(camHandle);
  is_ExitCamera(camHandle);
  std::cout << "Camera " << camHandle << " closed!" << std::endl;

  return 0;
}
