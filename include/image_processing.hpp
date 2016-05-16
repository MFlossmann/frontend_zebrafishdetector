#pragma once

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>

const double TAU = 6.283185307;
// openCV doesn't follow the mathematicial standard that positive angles are counterclockwise => (-360.0)
#define CV_DEG2RAD(x) (x * TAU/(-360.0))

const int TEMPLATE_H_CELLS = 4;
const int TEMPLATE_V_CELLS = 2;
const float DISH_H_CELLS = 6.;
const float DISH_V_CELLS = 4.;
const int CELL_WIDTH = 60;

using namespace cv;

struct imProcOptions{
  int cannyMaxThreshold;
  int centerDetectionThreshold;
};

// D-----C
// |     |
// A-----B
// struct orientedBoundingBox{
//   orientedBoundingBox();
//   orientedBoundingBox(boundingRect bounding_rect);

//   std::vector<Point> edges;

//   Point* A,B,C,D;

//   Point2f center;

//   double angle;

//   Size2f size;
// };
//typedef orientedBoundingBox obb;


//namespace imgproc{
//Mat createTemplate(Mat ringMatrix);

/*! \brief Matches the template in the image.

    \param image The image containing the template
    \param templ The template to look for
    \param found Whether the algorithm thinks it's found the dish or not (FIXXME: To be implemented)
    \param dish_rectangle The rectangle where the dish was supposedly found
    \param matching_method The method of OpenCV used for template matching
    \param show_result Option to show a window with the matching result matrix

    \return The upper-right corner of the best fit for the template in the original image
 */

Point matchDishTemplate(const Mat &image,
                        const Mat &templ,
                        bool found,
                        Rect &dish_rectangle,
                        int matching_method,
                        bool show_result = false);

void populateDish(Mat &image,
                  Rect dish,
                  int kernel_size,
                  double intensity);

/*! \brief Detects the Larvae in the dish

    \param image The image of the dish
    \param threshold the threshold value for detection
*/
void detectLarvae(Mat &image,
                  int threshold);

void rotate(const Mat &src,
            Mat &dst,
            double angle);

void rotateToNewSystem(std::vector<std::vector<int> > &circle_buffer,
                       double angle_deg,
                       Rect new_system);

Point rotateToNewSystem(Point point,
                        double angle_deg,
                        Rect new_system);

void fillCircleBuffer(std::vector<vector <int> > &circle_buffer, // Array of 3-dimensional vectors
                      std::vector<Vec3f> circles,
                      int circles_found,
                      float min_distance);

std::vector<Point> getCenterVector(const std::vector<vector <int> > &circle_buffer);

void drawCircleBuffer(Mat &display,
                      const std::vector<vector <int> > &circle_buffer,
                      Scalar color);

void createGrid(Rect rectangle,
                std::vector<std::vector<Point> > &edges,
                std::vector<std::vector<Point> > &centers);

void symmetricalStretch(Rect &rectangle,
                        float factor_y,
                        float factor_x=1.0);

//}
