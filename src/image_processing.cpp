#include "image_processing.hpp"

// oriented_bounding_box::oriented_bounding_box(boundingRect bounding_rect){
//   cv::Point2f edges_raw[4];
//   bounding_rect.points(edges_raw);
//   edges.resize(4);
//   if(bounding_rect.size.width < bounding_rect.size.height){
//     edges[0] = edges_raw[1];
//     edges[1] = edges_raw[0];
//     edges[2] = edges_raw[3];
//     edges[3] = edges_raw[2];

//     size = Size2f(bounding_rect.size.y,
//                   bounding_rect.size.x);

//     angle = 90.0 + bounding_rect.angle;
//   }
//   else{
//     edges[0] = edges_raw[0];
//     edges[1] = edges_raw[3];
//     edges[2] = edges_raw[2];
//     edges[3] = edges_raw[1];

//     size = bounding_rect.size;

//     angle = bounding_rect.angle;
//   }

//   center = bounding_rect.center;
// }

void findDish(Mat &image,
              Rect){

}

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
                  Rect dish,
                  int kernel_size,
                  double intensity){
  Mat temp = Mat::zeros(image.rows,
                        image.cols,
                        image.type());
  std::cout << "Populating dish..." << std::endl;

  double cell_width = 0.5*(dish.width/DISH_H_CELLS + dish.height/DISH_V_CELLS);

  // Help variables to help that the larvae end up in a square inside the cylinders
  int b = cell_width * sqrt(0.5) * 0.9;
  int d = 0.5*(cell_width - b);

  // only odd-valued kernel_sizes are allowed
  kernel_size += 1 - (kernel_size % 2);

  std::vector<Point> larvae;

  for(int row = 0; row < DISH_V_CELLS; row++){
    for (int col = 0; col < DISH_H_CELLS; col++) {
// for testing: Same seed every time
      int seed = 5*row + 3*col + 1;
      Point larva;
      srand(seed);
      larva.x = rand() % b + d + cell_width * col;
      srand(seed + 1);
      larva.y = rand() % b + d + cell_width * row;

      larva += dish.tl();

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

void rotate(const Mat &src,
            Mat &dst,
            double angle)
{
  Point2f pt(src.cols/2., src.rows/2.);
  Mat r = getRotationMatrix2D(pt, angle, 1.0);
  warpAffine(src, dst, r, Size(src.cols, src.rows));
}

Point rotateToNewSystem(Point point,
                        double angle_deg,
                        Rect new_system){
  point = point - new_system.tl();
  Point pivot = 0.5*(new_system.br() - new_system.tl());

  point -= pivot;
  point = cv::Point(point.x*cos(CV_DEG2RAD(angle_deg)) -
                    point.y*sin(CV_DEG2RAD(angle_deg)),
                    point.x*sin(CV_DEG2RAD(angle_deg)) +
                    point.y*cos(CV_DEG2RAD(angle_deg)));
  point += pivot;

  return point;
}
