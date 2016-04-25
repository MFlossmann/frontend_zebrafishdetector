#pragma once

#include <iostream>
using std::cout;
using std::endl;

#include <opencv2/opencv.hpp>
#include <ueye.h>

//////////////////// defines
#define SUCCESS 0
#define FAILURE -1
#define WARNING 1


namespace cam{

//////////////////// consts
const int IMAGE_WIDTH = 1280;
const int IMAGE_HEIGHT = 1024;

//////////////////// structs
struct cameraOptions{
        cameraOptions(){
                imageWidth = IMAGE_WIDTH;
                imageHeight = IMAGE_HEIGHT;
                aoiWidth = imageWidth;
                aoiHeight = imageHeight;
                aoiPosX = 0;
                aoiPosY = 0;

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
        double exposure;
        unsigned int ringBufferSize;

        bool undistortImage;
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
};

int initialize(cameraOptions &cam_options);

int setOptions(cameraOptions &cam_options);

}
