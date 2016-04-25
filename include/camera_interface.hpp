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

/*! /brief Sets the trigger mode, the framerate, the exposure time
           and the color depth of the camera.

    /param cam_options includes the camera options

    /return SUCCESS or FAILURE
 */
int setOptions(cameraOptions &cam_options);

int setAOI(cameraOptions &cam_options);

int initBuffers(const cameraOptions &cam_options,
                cv::Mat &image,
                cv::Mat &greyImage,
                std::vector<char*> &imgPtrList,
                std::vector<int> &imgIdList);

void getCaptureStatus(UEYE_CAPTURE_STATUS_INFO capture_status_info);
}
