#pragma once

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>

const int TEMPLATE_H_CELLS = 4;
const int TEMPLATE_V_CELLS = 2;
const int DISH_H_CELLS = 6;
const int DISH_V_CELLS = 4;
const int CELL_WIDTH = 60;

using namespace cv;

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
                  int kernel_size,
                  double intensity);

/*! \brief Detects the Larvae in the dish

    \param image The image of the dish
    \param threshold the threshold value for detection
*/
void detectLarvae(Mat &image,
                  int threshold);

//}
