#include <string>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include <chrono>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camera_interface.hpp"
#include "xyPlotter.hpp"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace po = boost::program_options;

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;

using namespace cv;

//////////////////// Enums
enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

//////////////////// Consts
const string SERIAL_DEVICE = "/dev/ttyUSB0";
const xyBaudRate BAUD_RATE = xyBaudRate::BAUD_115200;

const int KEY_ESCAPE = 27;

const double FRAMERATE = 10.;
const double FLASHTIME = 0.2;
const double EXPOSURE = 4;

// For the calibration run
const int PARTS = 4;
const double MIN_X = 70;
const double MIN_Y = 170;
const double MAX_X = 250;
const double MAX_Y = 270;

const string CONFIG_PATH_DEFAULT = "../cfg/conf_calib.xml";

//////////////////// Globals

//////////////////// CLASSES ////////////////////
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

    void write(cv::FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints" << bwritePoints
                  << "Write_extrinsicParameters"   << bwriteExtrinsics
                  << "Write_outputFileName"  << outputFileName

                  << "Show_UndistortedImage" << showUndistorsed

                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay
                  << "Input" << input
           << "}";
    }
    void read(const cv::FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> bwritePoints;
        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        interprate();
    }
    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        inputType = CAMERA;

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;


        calibrationPattern = NOT_EXISTING;
        calibrationPattern = CHESSBOARD;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        cv::FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != cv::FileNode::SEQ )
            return false;
        cv::FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }
public:
  cv::Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool bwritePoints;         //  Write detected feature points
    bool bwriteExtrinsics;     // Write extrinsic parameters
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorsed;       // Show undistorted images after calibration
    string input;               // The input ->



    int cameraID;
    vector<string> imageList;
    int atImageList;
    cv::VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};

//////////////////// FUNCTIONS ////////////////////

static void read(const FileNode& node,
                 Settings& x,
                 const Settings& default_value = Settings());
bool runCalibrationAndSave(Settings& s,
                           Size imageSize,
                           Mat&  cameraMatrix,
                           Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );
static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
                                        const vector<vector<Point2f> >& imagePoints,
                                        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                        const Mat& cameraMatrix , const Mat& distCoeffs,
                                        vector<float>& perViewErrors);
static void calcBoardCornerPositions(Size boardSize,
                                     float squareSize,
                                     vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/);
static bool runCalibration(Settings& s,
                           Size& imageSize,
                           Mat& cameraMatrix,
                           Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints,
                           vector<Mat>& rvecs,
                           vector<Mat>& tvecs,
                           vector<float>& reprojErrs,
                           double& totalAvgErr);
// Print camera parameters to the output file
static void saveCameraParams(Settings& s,
                             Size& imageSize,
                             Mat& cameraMatrix,
                             Mat& distCoeffs,
                             const vector<Mat>& rvecs,
                             const vector<Mat>& tvecs,
                             const vector<float>& reprojErrs,
                             const vector<vector<Point2f> >& imagePoints,
                             double totalAvgErr );



string getBinPath() {
  char buff[PATH_MAX];
  ssize_t len = ::readlink("/proc/self/exe",
                           buff,
                           sizeof(buff) - 2); // leave one index space for the \0 character

  while (--len > 0 && buff[len] != '/'); // remove the filename

  buff[++len] = '\0';
  return string(buff);
} // getBinPath()

int parseOptions(int argc,
                 char* argv[],
                 Settings &settings){
  string binary_path, config_file_path;

// get binary path
  binary_path = getBinPath();
  if (binary_path.empty()){
    cout << "Error collecting binary path! Terminating" << endl;
    exit(EXIT_FAILURE);
  }

  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "Produce help message")
    ("config,c", po::value<string>(&config_file_path)->default_value(binary_path + CONFIG_PATH_DEFAULT), "Different configuration file.")
    ;

  po::variables_map variable_map;
  store(po::command_line_parser(argc, argv).
        options(generic).run(), variable_map);

  notify(variable_map);

  std::ifstream ifs(config_file_path.c_str(),
                    std::ifstream::in);

  {
    boost::filesystem::path tmp(config_file_path);

//FIXXXME: check if the file exists or not!
    if (!ifs.is_open()){
    cout << "can't open config file: " << config_file_path << endl;
    exit(EXIT_FAILURE);
    }
    else{
      ifs.close();

      cv::FileStorage file_storage(config_file_path,
                                   cv::FileStorage::READ);

      file_storage["Settings"] >> settings;
      file_storage.release();
    }
  }

  if (variable_map.count("help")) {
    cout << generic << "\n";
    exit(EXIT_FAILURE);
  }

  return 0;
} // parseOptions

static void read(const FileNode& node,
                 Settings& x,
                 const Settings& default_value)
{
  if(node.empty())
    x = default_value;
  else
    x.read(node);
}

static void calcBoardCornerPositions(Size boardSize,
                                     float squareSize,
                                     vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
  corners.clear();

  switch(patternType)
  {
  case Settings::CHESSBOARD:
  case Settings::CIRCLES_GRID:
    for( int i = 0; i < boardSize.height; ++i )
      for( int j = 0; j < boardSize.width; ++j )
        corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
    break;

  case Settings::ASYMMETRIC_CIRCLES_GRID:
    for( int i = 0; i < boardSize.height; i++ )
      for( int j = 0; j < boardSize.width; j++ )
        corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
    break;
  default:
    break;
  }
}

static bool runCalibration(Settings& s,
                           Size& imageSize,
                           Mat& cameraMatrix,
                           Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints,
                           vector<Mat>& rvecs,
                           vector<Mat>& tvecs,
                           vector<float>& reprojErrs,
                           double& totalAvgErr)
{

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

static void saveCameraParams(Settings& s,
                             Size& imageSize,
                             Mat& cameraMatrix,
                             Mat& distCoeffs,
                             const vector<Mat>& rvecs,
                             const vector<Mat>& tvecs,
                             const vector<float>& reprojErrs,
                             const vector<vector<Point2f> >& imagePoints,
                             double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;

    if( s.flag )
    {
        sprintf( buf, "flags: %s%s%s%s",
            s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}


static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
                                        const vector<vector<Point2f> >& imagePoints,
                                        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                        const Mat& cameraMatrix , const Mat& distCoeffs,
                                        vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}


//////////////////// MAIN FUNCTION ////////////////////
int main(int argc, char** argv){
  Settings settings;

  parseOptions(argc,
               argv,
               settings);

  std::vector<std::vector<cv::Point2f> > imagePoints;
  cv::Mat cameraMatrix, distCoeffs;
  cv::Size imageSize;
  int mode = settings.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
  clock_t prevTimestamp = 0;
  const cv::Scalar RED(0,0,255), GREEN(0,255,0);
  const char ESC_KEY = 27;


//////////////////// Init serial communication
  xyPlotter xy_plotter(SERIAL_DEVICE);

 if (xy_plotter.connect(BAUD_RATE) != XY_SUCCESS){
   cerr << "Error initiating serial communication! Terminating..." << endl;
   return -1;
 }
 else
   cout << "Serial communication initiated..." << endl;

#define SLEEP_TIME 100000000

xy_plotter.setFrameRate(FRAMERATE);
// xy_plotter.setFlashTime(FLASHTIME);

 xy_plotter.moveAbs(10,10);
 xy_plotter.goHome();



//////////////////// Connect to the camera
  cam::cameraOptions cam_options;
  cam_options.framerate = FRAMERATE;
  cam_options.framerateEff = FRAMERATE;
  cam_options.flashTime = FLASHTIME;
  cam_options.exposure = EXPOSURE;
  cam_options.ringBufferSize = 1;
  cam_options.undistortImage = false;

  cam_options.triggerLevel = CAM_TRIGGER_RISING_EDGE;

  int status = 0;
  status |= cam::initialize(cam_options);
  status |= cam::setOptions(cam_options);
  status |= cam::initBuffers(cam_options);

  if (status != CAM_SUCCESS)
    return -1;

  int currentImgIndex = -1;
  char* currentImgPtr = 0;
  int currentImgId = -1;


  cv::Mat greyImage = Mat(cam_options.aoiHeight,
                          cam_options.aoiWidth,
                          CV_8UC1);
  cv::Mat image = Mat(cam_options.aoiHeight,
                      cam_options.aoiWidth,
                      CV_8UC3);
//////////////////// Main camera loop
  double delta_x = MAX_X - MIN_X;
  double delta_y = MAX_Y - MIN_Y;
  double inc_x = delta_x / (double)PARTS;
  double inc_y = delta_y / (double)PARTS;

  xy_plotter.moveAbs(MIN_X,
                     MIN_Y);

  for(int i=0;i<PARTS;i++){
    for(int j=0;j<PARTS;j++){
      int captureStatus;
      ++currentImgIndex %= cam_options.ringBufferSize;
      currentImgPtr = cam_options.imgPtrList[currentImgIndex];
      currentImgId = cam_options.imgIdList[currentImgIndex];


      is_SetImageMem(cam_options.camHandle,
                     currentImgPtr,
                     currentImgId);

      captureStatus = is_FreezeVideo(cam_options.camHandle,
                                     IS_WAIT);
      switch (captureStatus) {
      case IS_SUCCESS:
        break;
      case IS_TIMED_OUT:
        cerr << "Error capturing image! Timeout!" << endl;
      default:
        cerr << "Error capturing image! Error code: " << captureStatus << endl;
        break;
      } // switch captureStatus (FreezeVideo)


// copy the image to a opencv-manageable format.
      is_CopyImageMem(cam_options.camHandle,
                      currentImgPtr,
                      currentImgId,
                      //(char *) greyImage.data);
                      reinterpret_cast<char*>(greyImage.data));

      // Release the image buffer
      captureStatus |= is_UnlockSeqBuf(cam_options.camHandle,
                                       currentImgIndex,
                                       currentImgPtr);

// If enough data is available, stop calibration and show result.
      if( mode == CAPTURING && imagePoints.size() >= (unsigned)settings.nrFrames )
      {
        cout << "Enough data points collected. Calibrating..." << endl;
        // if( cv::runCalibrationAndSave(settings,
        //                               imageSize,
        //                               cameraMatrix,
        //                               distCoeffs,
        //                               imagePoints))
        //  mode = CALIBRATED;
        // else
        //  mode = DETECTION;
      }

      imageSize = image.size();
      if (settings.flipVertical)
        cv::flip(image,
                 image,
                 0);

      std::vector<cv::Point2f> pointBuffer;
      bool found;

// get the chessboard features
      found = cv::findChessboardCorners(greyImage,
                                        settings.boardSize,
                                        pointBuffer,
                                        CV_CALIB_CB_ADAPTIVE_THRESH |
                                        CV_CALIB_CB_FAST_CHECK |
                                        CV_CALIB_CB_NORMALIZE_IMAGE);

// Copy the grey image to a coloured one.
      image = greyImage.clone();
      cv::cvtColor(image,
                   image,
                   CV_GRAY2RGB);

// if Chessboard was found
      if (found){

// improve the found corners' coordinate accuracy
        cv::cornerSubPix(greyImage,
                         pointBuffer,
                         cv::Size(11,11),
                         cv::Size(-1,-1),
                         cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,
                                          30, 0.1));

// Draw the corners
        cv::drawChessboardCorners(image,
                                  settings.boardSize,
                                  cv::Mat(pointBuffer),
                                  found);
      }

      cv::namedWindow("Image View",
                      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
      cv::imshow("Image View",
                 image);

      xy_plotter.moveRelX(inc_x * pow(-1.0,(double) i));
    }
    xy_plotter.moveRelY(inc_y);
  }// while (cv::waitKey(10) != KEY_ESCAPE); // image capture loop.
}
