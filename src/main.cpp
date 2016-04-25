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
#include "camera_interface.hpp"

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

const double DEFAULT_FRAMERATE = 15.0;
const double DEFAULT_EXPOSURE_MS = 4;
const int IMAGE_TIMEOUT = 500;
const int TIMEOUT_COUNTER_MAX = 10;

const int RINGBUFFER_SIZE_DEFAULT = 10;
const std::string WINDOW_NAME = "Display Image (ESC to close)";

const cv::Scalar YELLOW = cv::Scalar(0,0xFF,0xFF);
const cv::Scalar RED = cv::Scalar(0,0,0xFF);

const int KEY_ESCAPE = 27;

//////////////////// structs


//////////////////// globals
cam::cameraOptions cam_options_;

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
    ("undistort,u", po::value<bool>(&cam_options_.undistortImage)->default_value(false), "undistort the image?")
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

  if(cam_options_.undistortImage)
    readCalibrationParameters(cam_options_.cameraMatrix,
                              cam_options_.distCoeffs);


  if (cam::initialize(cam_options_) != SUCCESS)
    return -1;

  if (cam::setOptions(cam_options_) != SUCCESS)
    return -1;

  if (cam::setAOI(cam_options_) != SUCCESS)
    return -1;

  cv::Mat image, greyImage;
  std::vector<char*> imgPtrList;
  std::vector<int> imgIdList;

  cam::initBuffers(cam_options_,
                   image,
                   greyImage,
                   imgPtrList,
                   imgIdList);

  // Activate the image queue
  is_InitImageQueue(cam_options_.camHandle,
                    0); // 0 is the only nMode supported


// enable the event that a new image is available
  is_EnableEvent(cam_options_.camHandle,
                 IS_SET_EVENT_FRAME);

  // enable video capturing
  is_CaptureVideo(cam_options_.camHandle, IS_WAIT);

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
      captureStatus = is_WaitForNextImage(cam_options_.camHandle,
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
        is_CaptureStatus(cam_options_.camHandle,
                         IS_CAPTURE_STATUS_INFO_CMD_GET,
                         (void*)&CaptureStatusInfo,
                         sizeof(CaptureStatusInfo));
        cam::getCaptureStatus(CaptureStatusInfo);
        // is_CaptureStatus(cam_options_.camHandle,
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
      captureStatus = is_CopyImageMem(cam_options_.camHandle,
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

      captureStatus |= is_UnlockSeqBuf(cam_options_.camHandle,
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

      if(cam_options_.undistortImage){

        cv::Mat tmp = image.clone();

        cv::undistort(tmp,
                      image,
                      cam_options_.cameraMatrix,
                      cam_options_.distCoeffs);
      } // undistortImage

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

// FIXXME: Still not sure if these are the proper fps
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
  is_StopLiveVideo(cam_options_.camHandle,
                   IS_FORCE_VIDEO_STOP);
  is_ClearSequence(cam_options_.camHandle);

  // Free image buffers
  for (int i=0; i < cam_options_.ringBufferSize; i++){
    is_FreeImageMem(cam_options_.camHandle, imgPtrList[i], imgIdList[i]);
  }
  imgPtrList.clear();
  imgIdList.clear();

  // Close events, buffers, queues and camera
  is_DisableEvent(cam_options_.camHandle,IS_SET_EVENT_FRAME);
  is_ExitImageQueue(cam_options_.camHandle);
  is_ClearSequence(cam_options_.camHandle);
  is_ExitCamera(cam_options_.camHandle);
  std::cout << "Camera " << cam_options_.camHandle << " closed!" << std::endl;

  return 0;
}
