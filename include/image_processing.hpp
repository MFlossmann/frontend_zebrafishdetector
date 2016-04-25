#pragma once

#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;

//namespace imgproc{
        Point matchTemplate(const Mat &image,
                            const Mat &templ,
                            int matching_method);

//}
