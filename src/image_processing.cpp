#include "image_processing.hpp"

Point matchTemplate(const Mat &image,
                    const Mat &templ,
                    int match_method){
        Mat image_display, result;
        image.copyTo(image_display);

        // Create the result matrix
        int result_cols = image.cols - templ.cols + 1;
        int result_rows = image.rows - templ.rows + 1;

        result = Mat(result_rows,
                     result_cols,
                     CV_32FC1);

        // Do the Matching and Normalize
        matchTemplate(image,
                      templ,
                      result,
                      match_method);
        normalize(result,
                  result,
                  0,
                  1,
                  NORM_MINMAX,
                  -1,
                  Mat());


        double max_val, min_val;
        Point max_loc, min_loc;

        minMaxLoc(result,
                  &min_val,
                  &max_val,
                  &min_loc,
                  &max_loc);

        // For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
        if ( match_method == CV_TM_SQDIFF ||
             match_method == CV_TM_SQDIFF_NORMED )
                return min_loc;
        else
                return max_loc;
}
