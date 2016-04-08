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

const int BUFFER_AMOUNT = 1;

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
    cout << "Error initializing, can't access the camera. Is it connected?" << endl;
    return -1;
  default:
    std::cout << "Error initializing camera. Err no. " << cameraStatus << std::endl;
    return -1;
  } // switch cameraStatus

  //////////////////// Set camera options
  
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
  

  //////////////////// Init the image buffers
  cv::Mat image(IMAGE_HEIGHT,IMAGE_WIDTH, COLOR_DEPTH_CV);
  std::vector<char*> imgPtrList;
  std::vector<int> imgIdList;

  imgPtrList.resize(BUFFER_AMOUNT);
  imgIdList.resize(BUFFER_AMOUNT);

  for (int i = 0; i < BUFFER_AMOUNT; i++){
    // Allocate memory for the bitmap-image
    bool error = IS_SUCCESS != is_AllocImageMem(camHandle,
						IMAGE_WIDTH,
						IMAGE_HEIGHT,
						COLOR_DEPTH,
						&imgPtrList[i],
						&imgIdList[i]);
    // TODO: Check, whether this is actually necessary at the beginning.
    /*    error |= IS_SUCCESS != is_SetImageMem(camHandle,
					  imgPtrList[i],
					  imgIdList[i]);*/
    if (error){
      std::cout << "Failed to initialize image buffer" << i << "." << std::endl;
      return -1;
    }

    std::cout << "Image buffer " << i << " allocated." << std::endl;
  }

  is_SetDisplayMode(camHandle, IS_SET_DM_DIB);
  is_SetImageMem(camHandle, imgPtrList[0], imgIdList[0]); // activate the imagebuffer

  do {
  // Get the image
  is_FreezeVideo(camHandle, IS_WAIT);

#ifdef _COPY_IMAGE_BUFFER_ // deepcopy the whole matrix
  // copy the image memory to the openCV memory
  is_CopyImageMem(camHandle,
		  imgPtrList[0],
		  imgIdList[0],
		  (char*)image.data);
#else // only copy the pointer
  image.data = (uchar*) imgPtrList[0];
#endif

  cv::namedWindow("Display Image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
  cv::imshow("Display Image", image);

  }
  while(cv::waitKey((int) (1000.0 / camOptions.framerateEff)) == -1);
  
  //    is_SetDisplayMode(camHandle, );
  
  // Free image buffers
  for (int i=0; i < BUFFER_AMOUNT; i++){
    is_FreeImageMem(camHandle, imgPtrList[i], imgIdList[i]);
  }
  imgPtrList.clear();
  imgIdList.clear();
  
  is_ExitCamera(camHandle); // close the camera
  std::cout << "Camera " << camHandle << " closed!" << std::endl;

  return 0;
} 

