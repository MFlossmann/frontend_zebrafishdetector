#include "image_processing.hpp"

void findDish(Mat &image,
              Rect){

}

// Mat createTemplate(Mat ringMatrix){
//   Mat templ(CELL_WIDTH*TEMPLATE_V_CELLS,
//             CELL_WIDTH*TEMPLATE_H_CELLS,
//             CV_8UC1,
//             Scalar(255));

//   for (int row = 0; row < TEMPLATE_V_CELLS; row++){
//     for (int col = 0; col < TEMPLATE_V_CELLS; col++){
//       if(ringMatrix.at<int>(row,col)){
//         int center_x = CELL_WIDTH * col + CELL_WIDTH/2;
//         int center_y = CELL_WIDTH * row + CELL_WIDTH/2;

//         circle(templ,
//                Point(center_x,
//                      center_y),
//                CELL_WIDTH/2,
//                Scalar(0),
//                1);
//       }
//     } // col
//   } // row

//   return templ;
// } // createTemplate

Point matchDishTemplate(const Mat &image,
                        const Mat &templ,
                        bool found,
                        Rect &dish_rectangle,
                        int match_method,
                        bool show_result){
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

  if (show_result){
    namedWindow("Matching result",
                CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    imshow("Matching result",
           result);
  }

  double max_val, min_val;
  Point max_loc, min_loc;

  minMaxLoc(result,
            &min_val,
            &max_val,
            &min_loc,
            &max_loc);

  Point best_match;

  // For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if ( match_method == CV_TM_SQDIFF ||
       match_method == CV_TM_SQDIFF_NORMED )
    best_match = min_loc;
  else
    best_match = max_loc;


  dish_rectangle = Rect(Point(best_match.x - CELL_WIDTH,
                              best_match.y - CELL_WIDTH),
                        Size(CELL_WIDTH*DISH_H_CELLS,
                             CELL_WIDTH*DISH_V_CELLS));

  return best_match;
}

void populateDish(Mat &image,
                  int kernel_size,
                  double intensity){
  Mat temp = Mat::zeros(image.rows,
                         image.cols,
                         image.type());
  // Help variables to help that the larvae end up in a square inside the cylinders
  int b = CELL_WIDTH * sqrt(0.5);
  int d = 0.5 * CELL_WIDTH * (1 - sqrt(0.5));

  // only odd-valued kernel_sizes are allowed
  kernel_size += 1 - (kernel_size % 2);

  std::vector<Point> larvae;

  for(int row = 0; row < DISH_V_CELLS; row++){
    for (int col = 0; col < DISH_H_CELLS; col++) {
// for testing: Same seed every time
      int seed = 5*row + 3*col + 1;
      Point larva;
      srand(seed);
      larva.x = rand() % b + d + CELL_WIDTH * col;
      srand(seed + 1);
      larva.y = rand() % b + d + CELL_WIDTH * row;

      larvae.push_back(larva);

      circle(temp,
             larva,
             1,
             Scalar(255,255,255),
             -1);
    } // for col
  } // for row

  // Blur the larvae
  GaussianBlur(temp,
               temp,
               Size(kernel_size, kernel_size),
               0,0);

  image = image - intensity*temp;
}

void detectLarvae(Mat &image,
                  int threshold){
  cv::threshold(image,
                image,
                threshold,
                0xFF,
                CV_THRESH_BINARY);
}
