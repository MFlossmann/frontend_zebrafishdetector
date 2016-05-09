/*
  Modeled after this homepage:
  https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual/index.html?is_initimagequeue.html
*/

#include <iostream>
#include <stdio.h>
#include <string>
#include <chrono>
#include <unistd.h>

//#include <opencv/cv.hpp>
#include "camera_interface.hpp"
#include "image_processing.hpp"

#include "xyPlotter.hpp"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace po = boost::program_options;
using std::cout;
using std::cerr;
using std::endl;

//////////////////// typedefs
typedef std::chrono::high_resolution_clock Time;

//////////////////// consts
const std::string CONFIG_FILE_DEFAULT = "../cfg/front_end.cfg";
// FIXXXME: Put into the proper folder!
const std::string TEMPLATE_FILE_DEFAULT = "../../rings.png";
const std::string UNDISTORT_FILE_DEFAULT = "../cfg/camera_data.yml";

const double DEFAULT_FRAMERATE = 1.0;
const double DEFAULT_EXPOSURE_MS = 1.0;
const double DEFAULT_FLASH_TIME = 0.20;
const int IMAGE_TIMEOUT = 500;
const int TIMEOUT_COUNTER_MAX = 10;

const int RINGBUFFER_SIZE_DEFAULT = 10;
const std::string WINDOW_NAME = "Display Image (ESC to close)";

const int CANNY_MAX_DEFAULT = 80;

const cv::Scalar YELLOW = cv::Scalar(0,0xFF,0xFF);
const cv::Scalar RED = cv::Scalar(0,0,0xFF);

const int KEY_ESCAPE = 27;

const xyBaudRate BAUD_RATE = xyBaudRate::BAUD_115200;

//////////////////// structs
struct simulationOptions{
  bool sim_mode;
  string image_path;
};

//////////////////// globals
cam::cameraOptions cam_options_;
std::string template_path_;
imProcOptions improc_options_;
simulationOptions sim_options_;
bool stay;

int high_threshold = 100;
double max_radius = CELL_WIDTH * 0.9;
double min_radius = CELL_WIDTH * 0.4;
double alpha = 1.0;
double beta = 30;

int high_threshold_slider, alpha_slider, beta_slider, max_radius_slider, min_radius_slider;

const int high_threshold_max = 255;
const int max_radius_max = 100;
const int min_radius_max = 100;
const int alpha_max = 100;
const int beta_max = 100;


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
    ("image,i", po::value<std::string>(&sim_options_.image_path)->default_value("/null"), "Only use the image instead of the camera")
    ("stay,s", "Don't initalize the plotter coordinates.")
    ;

  // Options that are allowed both on command line and in the config file
  po::options_description config("Configuration");
  config.add_options()
    ("framerate,f", po::value<double>(&cam_options_.framerate)->default_value(DEFAULT_FRAMERATE), "set framerate (fps)")
    ("exposure,e", po::value<double>(&cam_options_.exposure)->default_value(DEFAULT_EXPOSURE_MS), "set exposure time (ms)")
    ("buffer_size,b", po::value<unsigned int>(&cam_options_.ringBufferSize)->default_value(RINGBUFFER_SIZE_DEFAULT), "size of image ringbuffer")
    ("undistort,u", po::value<bool>(&cam_options_.undistortImage)->default_value(true), "undistort the image?")
    ("template,t", po::value<std::string>(&template_path_)->default_value(binary_path + TEMPLATE_FILE_DEFAULT), "Template file.")
    ("canny,m", po::value<int>(&improc_options_.cannyMaxThreshold)->default_value(CANNY_MAX_DEFAULT), "Max Threshold for the canny edge filter (circle detection)")
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


  if (variable_map.count("help")) {
    cout << cmdline_options << "\n";
    return 1;
  }

  if (variable_map.count("stay"))
    stay = true;
  else
    stay = false;

  sim_options_.sim_mode = (sim_options_.image_path.compare("/null") != 0);
  if (sim_options_.sim_mode){
    if (sim_options_.image_path[0] != '/')
      sim_options_.image_path = argv[0] + sim_options_.image_path;

    cout << "Simulation mode is active!" << endl;
    cout << "Sim pic path: " << sim_options_.image_path << endl;
  }
  return 0;
}

void readCalibrationParameters(cv::Mat &cameraMatrix,
                               cv::Mat &distCoeffs){
  // FIXXXXXME: make this dynamic!!
  cv::FileStorage fs(getBinPath() + UNDISTORT_FILE_DEFAULT, cv::FileStorage::READ);

  if (!fs.isOpened()){
    cout << "Couldn't open a recalibration file. Image will be distorted." << endl;

    return;
  }

  fs["Camera_Matrix"] >> cameraMatrix;
  fs["Distortion_Coefficients"] >> distCoeffs;

  return;
}

void on_trackbar( int, void*){
  alpha = (double) (25)*alpha_slider/alpha_max;
  beta = (double) 100*beta_slider/beta_max;

  max_radius = (double) CELL_WIDTH*max_radius_slider / max_radius_max;
  min_radius = (double) CELL_WIDTH*min_radius_slider / min_radius_max;

  high_threshold = high_threshold_slider;
}

int main(int argc, char* argv[]){
  xyPlotter xy_plotter("/dev/ttyUSB0");

  cv::Mat display;

// return if --help option was called
  if (parseOptions(argc, argv))
    return 1;

  bool simulation = sim_options_.sim_mode;

  //cam_options_.captureMode = cam::captureModeEnum::HARDWARE_LIVE;
  cam_options_.captureMode = cam::captureModeEnum::HARDWARE_LIVE;
  cam_options_.flashTime = DEFAULT_FLASH_TIME;

  if(cam_options_.undistortImage)
    readCalibrationParameters(cam_options_.cameraMatrix,
                              cam_options_.distCoeffs);

// Initiate serial communication.
  if(!simulation){
    if (xy_plotter.connect(BAUD_RATE) != XY_SUCCESS){
      cerr << "Warning! Error initiating serial communication! Continuing..." << endl;
    }
    else
      cerr << "Serial communication initiated." << endl;

    if(!stay){
      xy_plotter.goHome();
      xy_plotter.moveAbs(200,
                       50);
    }

// Set arduino controlled pparameters of the image capturing
    xy_plotter.setFrameRate(cam_options_.framerate);
    xy_plotter.setFlashTime(cam_options_.flashTime);

// Initialize the camera
    if (cam::initialize(cam_options_) != CAM_SUCCESS)
      return -1;

    cam::initBuffers(cam_options_);

    if (cam::setOptions(cam_options_) != CAM_SUCCESS)
      return -1;

    // if (cam::setAOI(cam_options_) != CAM_SUCCESS)
    //   return -1;



    cam::startVideoCapture(cam_options_);
  }
  else{
    cv::Mat tmp = cv::imread(sim_options_.image_path);

    cam_options_.imageHeight = tmp.rows;
    cam_options_.imageWidth = tmp.cols;
  }

//////////////////// Define trackbars ////////////////////
//  std::string img_proc_win_name = "processed image";
  high_threshold_slider = 55;
  alpha_slider = 6;
  beta_slider = 11;
  max_radius_slider = 63;
  min_radius_slider = 6;

  namedWindow(WINDOW_NAME,
              CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
  createTrackbar("alpha",
                 WINDOW_NAME,
                 &alpha_slider,
                 alpha_max,
                 on_trackbar);
  createTrackbar("beta",
                 WINDOW_NAME,
                 &beta_slider,
                 beta_max,
                 on_trackbar);
  createTrackbar("High Threshold",
                 WINDOW_NAME,
                 &high_threshold_slider,
                 high_threshold_max,
                 on_trackbar);
  createTrackbar("Max Radius",
                 WINDOW_NAME,
                 &max_radius_slider,
                 max_radius_max,
                 on_trackbar);
  createTrackbar("Min Radius",
                 WINDOW_NAME,
                 &min_radius_slider,
                 min_radius_max,
                 on_trackbar);

//////////////////// Image capture loop ////////////////////
  cv::Mat image = cv::Mat(cam_options_.imageHeight,
                          cam_options_.imageWidth,
                          CV_8UC3);

  cv::Mat greyImage = cv::Mat(cam_options_.aoiHeight,
                              cam_options_.imageWidth,
                              CV_8UC1);


  std::chrono::duration<float> frameDelta;
  auto currentTime = Time::now(),
    previousTime=Time::now();

  int currentImgIndex = -1;

//////////////////// Capture the image
  cv::Mat dish;

  int captureStatus;
  do{
    if(!simulation){
      ++currentImgIndex %= cam_options_.ringBufferSize;

      int timeoutCounter = 0;
      do{
        captureStatus = cam::getImage(cam_options_,
                                      IMAGE_TIMEOUT,
                                      currentImgIndex);

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

// copy the image memory to the openCV memory
        // captureStatus =
        is_CopyImageMem(cam_options_.camHandle,
                        cam_options_.imgPtrList[currentImgIndex],
                        cam_options_.imgIdList[currentImgIndex],
                        reinterpret_cast<char*>(greyImage.data));
        //(char*) greyImage.data);
        //greyImage.data = reinterpret_cast<uchar*>(cam_options_.imgPtrList[currentImgIndex]);

        captureStatus |= freeBuffer(cam_options_,
                                    currentImgIndex);

        if(captureStatus != IS_SUCCESS)
          cout << "Error copying buffer: " << captureStatus << endl;
#ifdef _VERBOSE_MODE_
        else
          cout << "Buffer " << currentImgIndex << " copied and released.\n";
#endif // _VERBOSE_MODE_

//////////////////// Undistort the image

        if(cam_options_.undistortImage){
          cv::Mat tmp = greyImage.clone();

          cv::undistort(tmp,
                        greyImage,
                        cam_options_.cameraMatrix,
                        cam_options_.distCoeffs);
        } // undistortImage
      } // captureStatus == IS_SUCCESS

//////////////////// No new image received
      else {
        cout << "No image received, copying the old one." << endl;

        // cv::putText(image,
        //             "Image out of date!",
        //             cv::Point(image.cols*.1,
        //                       image.rows*.94),
        //             CV_FONT_HERSHEY_PLAIN,
        //             5.0,
        //             RED,
        //             3);
      } // captureStatus != IS_SUCCESS

    } // end if simulation
    else
    {
      greyImage = cv::imread(sim_options_.image_path,
                             CV_8UC1);
    }
	cv::Mat processing_image;
    // cv::GaussianBlur(greyImage,
    //                  processing_image,
    //                  cv::Size(7,7),
    //                  0);
  cv::medianBlur(greyImage,
                 processing_image,
                 5);

//  double alpha = 3.0;
//  double beta = -30;

  processing_image = alpha*processing_image - beta;

  processing_image.copyTo(display);
	cv::cvtColor(display,
                 display,
                 CV_GRAY2RGB);


//////////////////// Template matching
    cv::Mat templ = imread(template_path_);
    cv::Rect dish_rectangle;

    bool found = false;
    cv::Point match_point;
    // cv::Point match_point = matchDishTemplate(display,
    //                                           templ,
    //                                           found,
    //                                           dish_rectangle,
    //                                           CV_TM_CCOEFF,
    //                                           true);

//////////////////// Circle detection
//    int high_threshold = improc_options_.cannyMaxThreshold;
//    double max_radius = CELL_WIDTH * 0.9;
//    double min_radius = CELL_WIDTH * 0.4;

    std::vector<Vec3f> circles;
    HoughCircles(processing_image,
                 circles,
                 CV_HOUGH_GRADIENT,
                 2,
                 max_radius*0.5,
                 high_threshold,
                 100,
                 min_radius,
                 max_radius);
    cout << "Amount of circles detected: " << circles.size() << endl;

    for (size_t i = 0; i < circles.size(); i++){
      cv::Point center(cvRound(circles[i][0]),
                       cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      circle(display,center, 3, Scalar(0,255,0), -1, 8, 0);
      circle(display, center, radius, Scalar(0,0,255), 3, 8, 0);
    }
    circle(display, cv::Point(display.cols * 0.9, display.rows * 0.9),
           max_radius, cv::Scalar(255,0,0), 3, 8, 0);

    circle(display, cv::Point(display.cols * 0.9, display.rows * 0.9),
           min_radius, cv::Scalar(255,0,0), 3, 8, 0);

    ////////// Crop the dish
    if (found){
      cv::Mat croppedReference(display,
                               dish_rectangle);
      croppedReference.copyTo(dish);

      populateDish (dish,
                    5,
                    0.25);
      detectLarvae (dish,
                    77);

      cv:: rectangle (display,
                      match_point,
                      cv::Point(match_point.x + templ.cols,
                                match_point.y + templ.rows),
                      RED,
                      2,
                      8,
                      0);

      int cell_size = templ. rows / 2;

      cv:: rectangle (display,
                      dish_rectangle,
                      YELLOW,
                      2,
                      8,
                      0);
    }

// FIXXME: Kind of sure these are the proper fps
//////////////////// FPS display
    currentTime = Time::now();
    frameDelta = currentTime - previousTime;
    previousTime = currentTime;
    float fps = 1/frameDelta.count();

    std::stringstream fpsDisplay;
    fpsDisplay << "FPS: " << fps;

    cv::putText(display,
                fpsDisplay.str().c_str(), // stringstream -> string -> c-style string
                cv::Point(150,50),
                CV_FONT_HERSHEY_PLAIN,
                2.0,
                YELLOW);
// END FIXXME



//////////////////// Display the image

    cv::imshow(WINDOW_NAME,
               display);

    cv::namedWindow("Image capture",
                    CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
    cv::imshow("Image capture",
               greyImage);

    if (found){
      cv::namedWindow("Dish",
                      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
      cv::imshow("Dish",
                 dish);
    }

  }while(cv::waitKey(10) != KEY_ESCAPE);

//////////////////// Cleanup and exit

//TODO: Return status of closing
  is_StopLiveVideo(cam_options_.camHandle,
                   IS_FORCE_VIDEO_STOP);
  is_ClearSequence(cam_options_.camHandle);

// Free image buffers
  for (int i=0; i < cam_options_.ringBufferSize; i++){
    is_FreeImageMem(cam_options_.camHandle,
                    cam_options_.imgPtrList[i],
                    cam_options_.imgIdList[i]);
  }
  cam_options_.imgPtrList.clear();
  cam_options_.imgIdList.clear();

// Close events, buffers, queues and camera
  is_DisableEvent(cam_options_.camHandle,IS_SET_EVENT_FRAME);
  is_ExitImageQueue(cam_options_.camHandle);
  is_ClearSequence(cam_options_.camHandle);
  is_ExitCamera(cam_options_.camHandle);
  std::cout << "Camera " << cam_options_.camHandle << " closed!" << std::endl;

// Save the last image
  std::ostringstream image_path, date, processed_image_path;
  time_t t = time(0);
  struct tm * now = localtime(&t);
  image_path << "/tmp/"
             << "zebrafish";
  date << (now->tm_year + 1900) << '-';
  if ((now->tm_mon + 1) < 10)
    date << '0';
  date <<(now->tm_mon) + 1 << '-';
  if ((now->tm_mday) < 10)
    date << '0';
  date << now->tm_mday << '_'
       << now->tm_hour << now->tm_min << now->tm_sec;
  image_path << "/tmp/zebrafish_" << date.str() << ".png";
  processed_image_path << "/tmp/zebrafish_detected_" << date.str() << ".png";
  cv::imwrite(image_path.str(),
              greyImage);

  cv::imwrite(processed_image_path.str(),
              display);

  return 0;
}
