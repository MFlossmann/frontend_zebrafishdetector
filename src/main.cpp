/*
  Modeled after this homepage:
  https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual/index.html?is_initimagequeue.html
*/

#include <iostream>
#include <stdio.h>
#include <string>
#include <chrono>
#include <unistd.h>

#include <mutex>
#include <thread>

//#include <opencv/cv.hpp>
#include "camera_interface.hpp"
#include "image_processing.hpp"

#include "opencv2/video/tracking.hpp"

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

//////////////////// enums
enum detectionState{
  DISH_LOCATION,
  DISH_OBB,
  LARVAE_MOVEMENT
};

//////////////////// structs
struct xyPlotterPipe{
  std::mutex standing_still_mutex; // default: unlocked
  std::mutex goal_available_mutex; // default: locked
  std::mutex close_enough_mutex;   // default: locked
  cv::Point_<double> goal;
};

//////////////////// consts
const std::string CONFIG_FILE_DEFAULT = "../cfg/front_end.cfg";
// FIXXXME: Put into the proper folder!
const std::string TEMPLATE_FILE_DEFAULT = "../../pictures/template01.png";
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

const double MM_PER_PIXEL_MOVEMENT = 0.1;
const float CONFIDENCE_MIN = 30; // Empirical

const int CONFIDENCE_COUNTER_MIN = 4; // How long for the center to be in the sweet spot
const int OBB_COUNTER_MIN = 8; // How long to look for the OBB

const int MIN_FOUND_CIRCLES = 3;

const int CENTER_BUFFER_SIZE = 3*24;

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
double alpha = 1.75;
double beta = 44;

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

  cout << "\tα =\t" << alpha << endl
       << "\tβ =\t" << beta << endl
       << "\tmax_r =\t" << max_radius << endl
       << "\tmin_r =\t" << min_radius << endl
       << "threshold =\t" << high_threshold << endl;
}

std::vector<int> pointToVector(cv::Point point){
  std::vector<int> result(2);

  result[0] = point.x;
  result[1] = point.y;

  return result;
}

// FIXXME: Might be incompatible to ints
cv::Mat pointToMat(cv::Point point){
  cv::Mat result(2,1,CV_32F);

  result.at<float>(0) = point.x;
  result.at<float>(1) = point.y;
  return result;
}

cv::Point matToPoint(cv::Mat mat){
  // First: transpose possible row vectors
  if (mat.cols == 2 && mat.rows == 1)
    cv::transpose(mat,mat);

  if (mat.cols == 1 && mat.rows == 2){
    return cv::Point((int) mat.at<float>(0,0),
                     (int) mat.at<float>(0,1));
  }
  else{
    std::stringstream error;

    error << "Can't convert a matrix of dimension"
          << mat.rows << "x" << mat.cols
          << " to a point. It has to be a 2D vector!";
    throw std::runtime_error(error.str());
  }

}

void xyPlotterMove(xyPlotter &xy_plotter,
                   xyPlotterPipe &xy_plotter_pipe){
  while(true)
  {
    xy_plotter_pipe.goal_available_mutex.lock();
    cout << "\tGoal available!\n";

// thread can finish if close enough
    if( xy_plotter_pipe.close_enough_mutex.try_lock() ){
      // unlock all the mutexes to signal that the thread is finished
      xy_plotter_pipe.standing_still_mutex.unlock();
      xy_plotter_pipe.close_enough_mutex.unlock();

      break;
    }

    xy_plotter.moveRel(xy_plotter_pipe.goal.x,
                        xy_plotter_pipe.goal.y);
    xy_plotter_pipe.standing_still_mutex.unlock();
  }

  return;
}


std::vector<Vec3f> findDrawCircles(cv::Mat &processing_image,
                                   cv::Mat &display,
                                   int &circles_found,
                                   cv::Point *center_measurement = 0,
                                   bool draw_radius=false){
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

  if (center_measurement != 0)
    *center_measurement = cv::Point(0,0);

  circles_found = circles.size();
  for (size_t i = 0; i < circles_found; i++){
    cv::Point center(cvRound(circles[i][0]),
                     cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);

    circle(display, center, 3, Scalar(0,255,0), -1, 8, 0);

    if (draw_radius)
    circle(display, center, radius, Scalar(0,0,255), 3, 8, 0);

    if (center_measurement != 0)
      *center_measurement += center * ((double) 1/circles_found);
  }

  return circles;
}

void drawCrossHair(cv::Mat &image,
                   cv::Point center,
                   int radius,
                   cv::Scalar color){
  circle(image,
         center,
         radius,
         color,
         1);
  line(image,
       center + cv::Point(0,radius),
       center - cv::Point(0,radius),
       color,
       1);
  line(image,
       center + cv::Point(radius,0),
       center - cv::Point(radius,0),
       color,
       1);
    }

int main(int argc, char* argv[]){
// return if --help option was called
  if (parseOptions(argc, argv))
    return 1;

  xyPlotter xy_plotter("/dev/ttyUSB0");
  xyPlotterPipe xy_plotter_pipe;

//////////////////// Initialize Kalman filter
  KalmanFilter kalman_filter(2,2,2);
  // \bar{\mathbf{x}}_k = A*\mathbf{x}_{k-1} + B*\mathbf{u}_k + \mathbf{w}_k
  // Transition Matrix A
  cv::setIdentity(kalman_filter.transitionMatrix);
  // Control Matrix B
  cv::setIdentity(kalman_filter.controlMatrix);
  // Process noise vector w ~ N(Process noise covariance Matrix Q)
  cv::setIdentity(kalman_filter.processNoiseCov,
                  Scalar::all(1e-1));

  // \mathbf{z}_k=H*\mathbf{x}_k + \mathbf{v}_k
  // Matrix H
  cv::setIdentity(kalman_filter.measurementMatrix,
                  Scalar::all(1.0));
  // Vector v ~ N(measurement Noise covariance R_k)
  cv::setIdentity(kalman_filter.measurementNoiseCov,
                  Scalar::all(0));
  kalman_filter.measurementNoiseCov.at<float>(0,0) = 6e-1;
  kalman_filter.measurementNoiseCov.at<float>(1,1) = 4e-1;
  cv::Mat kalman_R = kalman_filter.measurementNoiseCov;

  cv::setIdentity(kalman_filter.errorCovPost,
                  Scalar::all(1));

  bool simulation = sim_options_.sim_mode;

//   cam_options_.captureMode = cam::captureModeEnum::HARDWARE_FREEZE;
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
      xy_plotter.moveAbs(150,
                         150);
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
  alpha_slider = 11;
  beta_slider = 44;
  max_radius_slider = 55;
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

//////////////////// FIXXXXXME: here comes the thread initialization
  cout << "Starting the xyPlotter moving thread...";
  std::thread xy_plotter_move(xyPlotterMove,
                              std::ref(xy_plotter),
                              std::ref(xy_plotter_pipe));
  xy_plotter_pipe.standing_still_mutex.unlock();
  xy_plotter_pipe.goal_available_mutex.lock();
  xy_plotter_pipe.close_enough_mutex.lock();

//////////////////// InitImage capture loop ////////////////////
  detectionState detection_state = detectionState::DISH_LOCATION;
  int confidence_counter = 0;
  int obb_counter = 0;

  float dish_angle = 0.0;

  // basically a FIFO buffer for the center points
  std::vector<cv::Point> cell_center_buffer;

  cv::Mat greyImage(cam_options_.imageHeight,
                    cam_options_.imageWidth,
                    CV_8UC1);

  cv::Mat processing_image, display;

  // init Kalman posterior belief
  cv::Point image_center(cam_options_.imageWidth/2,
                         cam_options_.imageHeight/2);

  randn(kalman_filter.statePost,
        cv::Scalar(image_center.x,
                   image_center.y),
        cv::Scalar::all(0));
        // cv::Scalar(image_center.x/2,
        //            image_center.y/2));

  // initialize the posterior_mat
  cv::Mat posterior_mat = kalman_filter.predict();

// FPS calculation
  std::chrono::duration<float> frameDelta;
  auto currentTime = Time::now(),
    previousTime=Time::now();

  int currentImgIndex = -1;

  cv::Mat dish;

  int captureStatus;

  cout << "Starting loop...\n"
       << "\tα = " << alpha << endl
       << "\tβ = " << beta << endl
       << "\tmax_r" << max_radius << endl
       << "\tmin_r" << min_radius << endl;
//////////////////// Image loop
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
        // captureStatus = is_CopyImageMem(cam_options_.camHandle,
        //                                 cam_options_.imgPtrList[currentImgIndex],
        //                                 cam_options_.imgIdList[currentImgIndex],
        //                                 reinterpret_cast<char*>(greyImage.data));

        //(char*) greyImage.data);
        greyImage.data = reinterpret_cast<uchar*>(cam_options_.imgPtrList[currentImgIndex]);

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

//////////////////////////////////////// Processing the image ////////////////////////////////////////
    cout << "cloning...";
    greyImage.copyTo(processing_image);
    cout << " cloned!\n";
    cv::medianBlur(processing_image,
                   processing_image,
                   5);

    processing_image = alpha*processing_image - beta;
    processing_image.copyTo(display);
    cv::cvtColor(display,
                 display,
                 CV_GRAY2RGB);

    switch (detection_state){
    case detectionState::DISH_LOCATION:{
//////////////////// Circle detection
      std::vector<Vec3f> circles;
      cv::Point center_measurement;
      int circles_found;

      circles = findDrawCircles(processing_image,
                                display,
                                circles_found,
                                &center_measurement,
                                true);

      circle(display, cv::Point(display.cols * 0.9, display.rows * 0.9),
             max_radius, cv::Scalar(255,0,0), 3, 8, 0);

      circle(display, cv::Point(display.cols * 0.9, display.rows * 0.9),
             min_radius, cv::Scalar(255,0,0), 3, 8, 0);

//////////////////// Kalman filter update

      cv::Point posterior = matToPoint(posterior_mat);

      // Kalman filter correction
      if(circles_found > MIN_FOUND_CIRCLES){

      // only correct the Kalman filter if the XY_Plotter is standing still
        if( xy_plotter_pipe.standing_still_mutex.try_lock() )
        {
          posterior_mat = kalman_filter.correct(pointToMat(center_measurement));

          posterior = matToPoint(kalman_filter.statePost);

//////////////////// Move the camera to center the dish
          double diff_x = (double) posterior.x - image_center.x;
          // different sign, because opencv counts from the upper right corner
          double diff_y = (double) image_center.y - posterior.y;
          cout << "Moving relatively: " << endl << "\t"
               << posterior.x << "-" << image_center.x << ","
               << posterior.y << "-" << image_center.y << " = ("
               << diff_x << ","
               << diff_y << ")" << endl;

          float confidence = pow(diff_x,2.0) + pow(diff_y,2.0);
          cout << "Confidence: " << confidence << endl;
          if (confidence <= pow(CONFIDENCE_MIN,2.0)){
            // make sure we're confident long enough
            if(confidence_counter++ >= CONFIDENCE_COUNTER_MIN){
              cout << "\tClose enough! Shutting down the mover...\n";
              xy_plotter_pipe.close_enough_mutex.unlock();

              cout << "\n\nSwitching to template matching...\n\n";
              detection_state = detectionState::DISH_OBB;
              break;
            } // if confidence_counter >= CONFIDENCE_CONTER_MIN
          } // if confidence <= CONFIDENCE_MIN
          else
            confidence_counter = 0;

          xy_plotter_pipe.goal = cv::Point_<double>(diff_x * MM_PER_PIXEL_MOVEMENT,
                                                    diff_y * MM_PER_PIXEL_MOVEMENT);
          posterior_mat = kalman_filter.predict(pointToMat(xy_plotter_pipe.goal));
          posterior = matToPoint(kalman_filter.statePre);

          cout << "Unlocking the mover...\n";
          xy_plotter_pipe.goal_available_mutex.unlock();
        } // if standing still
      }

      // display the dish posterior
      circle(display,
             posterior,
             5,
             cv::Scalar(0,0xFF,0xFF),
             -1);
      // display the measurement confidence
      cv::circle(display,
                 posterior,
                 CONFIDENCE_MIN,
                 cv::Scalar(0,0,0xFF),
                 3);
      // display goal
      drawCrossHair(display,
                    cv::Point(display.cols/2,
                              display.rows/2),
                    20,
                    cv::Scalar(0xFF,0,0));
    }
      break;
    case detectionState::DISH_OBB:{
      std::vector<Vec3f> circles;
      cv::Point center_measurement;
      int circles_found;

      circles = findDrawCircles(processing_image,
                                display,
                                circles_found);

      std::vector<cv::Point> centers;
      for(int i = 0; i < circles_found; i++){
        cell_center_buffer.push_back(cv::Point(cvRound(circles[i][0]),
                                               cvRound(circles[i][1])));

        if(cell_center_buffer.size() > CENTER_BUFFER_SIZE)
          cell_center_buffer.erase(cell_center_buffer.begin());
        }
      cv::RotatedRect oriented_bounding_box = minAreaRect(cell_center_buffer);

      cv::Point2f obb_points[4];
      oriented_bounding_box.points(obb_points);

      cv:Point2f obb_point_center = oriented_bounding_box.center;

      double LAMBDA = sqrt(26.0/17.0);

      for (int j=0; j < 4; j++){
        obb_points[j] = obb_point_center + LAMBDA * (obb_points[j] - obb_point_center);
      }

      for (int j=0; j < 4; j++)
        line(display,
             obb_points[j],
             obb_points[(j+1)%4],
             cv::Scalar(0xFF,0,0),
             3);

      if (obb_counter++ >= OBB_COUNTER_MIN){
        std::vector<cv::Point> obb_point_vector;
        for(int j = 0; j < 4; j++)
          obb_point_vector.push_back(obb_points[j]);

        cv::Rect bounding_rect = cv::boundingRect(obb_point_vector);

        dish_angle = oriented_bounding_box.angle;

        cam_options_.aoiWidth = bounding_rect.width;
        cam_options_.aoiHeight = bounding_rect.height;
        cam_options_.aoiPosX = bounding_rect.x;
        cam_options_.aoiPosY = bounding_rect.y;

        cam_options_.undistortImage = false;

        cout << "Renitialize the buffers...\n";
        cam::stop(cam_options_);

        cout << "Adjusting Area of interest...";
        cam::setAOI(cam_options_);

        cam::initBuffers(cam_options_);

        cam::startVideoCapture(cam_options_);

        display.release();
        display.create(cam_options_.aoiHeight,
                       cam_options_.aoiWidth,
                       CV_8UC3);

        processing_image.release();
        processing_image.create(cam_options_.aoiHeight,
                                cam_options_.aoiWidth,
                                CV_8UC1);

        greyImage.release();
        greyImage.create(cam_options_.aoiHeight,
                         cam_options_.aoiWidth,
                         CV_8UC1);

        detection_state = detectionState::LARVAE_MOVEMENT;
      }
    }
      break;
    case detectionState::LARVAE_MOVEMENT:{

    }
      break;
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

//////////////////// Display the image ////////////////////

    cv::imshow(WINDOW_NAME,
               display);

    cv::namedWindow("Image capture",
                    CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
    cv::imshow("Image capture",
               greyImage);

    // if (found){
    //   cv::namedWindow("Dish",
    //                   CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    //   cv::imshow("Dish",
    //              dish);
    // }

  }while(cv::waitKey(10) != KEY_ESCAPE);

//////////////////// Cleanup and exit

// Clean up the thread
  xy_plotter_move.detach();

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

  cout << "Saving grey image to " << image_path.str() << endl;;
  cv::imwrite(image_path.str(),
              greyImage);

  cout << "Saving processed image to " << processed_image_path.str() << endl;

  cv::imwrite(processed_image_path.str(),
              display);

  return 0;
}
