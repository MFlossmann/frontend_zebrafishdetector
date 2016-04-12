/*
Modeled after this homepage:
https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual/index.html?is_initimagequeue.html

current goal: video mode with freerun mode

 */

#include <iostream>
#include <stdio.h>
#include <string>
//#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <ueye.h>

#include <boost/program_options.hpp>
#include <fstream>

namespace po = boost::program_options;
using std::cout;
using std::endl;

//////////////////// consts
const std::string CONFIG_FILE_DEFAULT = "../cfg/front_end.cfg";

const int IMAGE_WIDTH = 1280;
const int IMAGE_HEIGHT = 1024;
const int COLOR_DEPTH = 8;
const int COLOR_DEPTH_CV = CV_8UC1;
const double DEFAULT_FRAMERATE = 30.0;
const double DEFAULT_EXPOSURE_MS = 4;
const int IMAGE_TIMEOUT = 500;
const int TIMEOUT_COUNTER_MAX = 10;


const int BUFFER_AMOUNT = 2;

//////////////////// structs
struct cameraOptions{
  cameraOptions() :
    framerate(DEFAULT_FRAMERATE),
    exposure(DEFAULT_EXPOSURE_MS){}
  double framerate;
  double framerateEff;
  double exposure;
};

//////////////////// globals
cameraOptions camOptions;

//////////////////// functions
int parseOptions(int argc, char* argv[]){
  std::string config_file;
  
  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "Produce help message")
    ("config,c", po::value<std::string>(&config_file)->default_value(CONFIG_FILE_DEFAULT), "Different configuration file.")
    ;
  
  // Options that are allowed both on command line and in the config file
  po::options_description config("Configuration");
  config.add_options()
    ("framerate,f", po::value<double>(), "set framerate (fps)")
    ("exposure,e", po::value<double>(), "set exposure time (ms)")
    ;
  
  po::options_description cmdline_options;
  cmdline_options.add(generic).add(config);
  
  po::options_description config_file_options;
  config_file_options.add(config);
  
  po::variables_map vm;
  store(po::command_line_parser(argc, argv).
	options(cmdline_options).run(), vm);
  
  notify(vm);

  std::ifstream ifs(config_file.c_str());
  // if (!ifs){
  //   cout << "can't open config file: " << config_file << endl;
  //   return 2;
  // }
  // else
  //   {
  //     store(parse_config_file(ifs, config_file_options), vm);
  //     notify(vm);
  //   }

  if (vm.count("help")) {
    cout << cmdline_options << "\n";
    return 1;
  }
  if (vm.count("framerate")){
    cout << "Custom framerate set." << endl;
    camOptions.framerate = vm["framerate"].as<double>();
  }
  if (vm.count("exposure")){
    cout << "Custom exposure set." << endl;
    camOptions.exposure = vm["exposure"].as<double>();
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

int main(int argc, char* argv[]){
// return if --help option was called
  if (parseOptions(argc, argv))
    return 1;
 
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
                        IS_SET_TRIGGER_SOFTWARE); // Set to SW trigger for freerun mode

  // Framerate
  cameraStatus = is_SetFrameRate(camHandle,
                                 camOptions.framerate,
                                 &camOptions.framerateEff);
  switch (cameraStatus){
  case IS_SUCCESS:
    cout << "Framerate was set to " << camOptions.framerate		\
	 << "fps, the effective framerate is " << camOptions.framerateEff \
	 << "fps." << endl;
    break;
  default:
    cout << "Error setting framerate. Err no. " << cameraStatus << endl;
  }

  // Exposure time set to specific value
  cameraStatus = is_Exposure(camHandle,
                             IS_EXPOSURE_CMD_SET_EXPOSURE,
                             &camOptions.exposure,
                             sizeof(camOptions.exposure));
  switch (cameraStatus){
  case IS_SUCCESS:
    cout << "Exposure time set to " << camOptions.exposure	\
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
  std::vector<char*> imgPtrList;
  std::vector<int> imgIdList;

  imgPtrList.resize(BUFFER_AMOUNT);
  imgIdList.resize(BUFFER_AMOUNT);


  is_SetDisplayMode(camHandle, IS_SET_DM_DIB); // TODO: Where to put this?

  for (int i = 0; i < BUFFER_AMOUNT; i++){
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

  do {

    char* currentImgPtr = 0;
    int currentImgIndex = -1;

//////////////////// Capture the image
    {
      int timeoutCounter = 0;
      int captureStatus;
      do{
        // TODO: This function fails (IS_NO_SUCCESS)
        captureStatus = is_WaitForNextImage(camHandle,
                                            IMAGE_TIMEOUT,
                                            &currentImgPtr,
                                            &currentImgIndex);

        switch(captureStatus){
        case IS_TIMED_OUT:
        case IS_SUCCESS:
          break;
        case IS_CAPTURE_STATUS:   // Specific error
          UEYE_CAPTURE_STATUS_INFO CaptureStatusInfo;
          is_CaptureStatus(camHandle,
                           IS_CAPTURE_STATUS_INFO_CMD_GET,
                           (void*)&CaptureStatusInfo,
                           sizeof(CaptureStatusInfo));
          parseCaptureStatus(CaptureStatusInfo);
          is_CaptureStatus(camHandle,
                           IS_CAPTURE_STATUS_INFO_CMD_RESET,
                           0,
                           0);
          goto afterCapture;  // Note: Breaking from a nested block is the single use for goto statements
        default:
          cout << "Error capturing image! Error code: " << captureStatus << endl;
          goto afterCapture;
        }
      }
      while (captureStatus != IS_TIMED_OUT ||
             ++timeoutCounter >= TIMEOUT_COUNTER_MAX);
    afterCapture:

      if (timeoutCounter >= TIMEOUT_COUNTER_MAX)
        cout << "Image timeout!" << endl;
    }

//////////////////// Copy the image buffer

#ifdef _COPY_IMAGE_BUFFER_ // deepcopy the whole matrix
    // copy the image memory to the openCV memory
    is_CopyImageMem(camHandle,
                    currentImgPtr,
                    currentImgIndex,
                    (char*)image.data);
#else // only copy the pointer
    image.data = (uchar*) currentImgPtr;
#endif

    cv::namedWindow("Display Image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
    cv::imshow("Display Image", image);

  }while(cv::waitKey((int) (1000.0 / camOptions.framerateEff)) == -1);

//////////////////// Cleanup and exit

  //TODO: Return variables
  is_StopLiveVideo(camHandle,
                   IS_FORCE_VIDEO_STOP);
  is_ClearSequence(camHandle);
  
  // Free image buffers
  for (int i=0; i < BUFFER_AMOUNT; i++){
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

