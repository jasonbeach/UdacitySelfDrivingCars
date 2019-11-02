#include "fmt/core.h" 
#include <string>
#include <opencv2/opencv.hpp>
#include "cv_helpers.hpp"


int main(int argc, char** argv ){
  std::string filename;
  if ( argc != 2 ){
    filename = "test.jpg";}
  else{
    filename = std::string(argv[1]);}
  
  cv::Mat image = cv::imread( filename, cv::IMREAD_COLOR );
  
  if ( !image.data ){
    fmt::print("No image data \n");
    return -1;}

  auto y_size = image.size[0];
  auto x_size = image.size[1];
  fmt::print("image size: {}x{} type: {}\n", x_size, y_size, type2str(image.type()));


  const cv::Point left_bottom{110, y_size};
  const cv::Point right_bottom{x_size - 110, y_size};
  const cv::Point apex {x_size/2, y_size/2+25};

  const std::vector<cv::Point> triangle{left_bottom, right_bottom, apex};
  const std::vector<std::vector<cv::Point>> contour_list{triangle};


  // create a blank image (black image)
  cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC3); 
  
  for(int i = 0; i < contour_list.size(); i++) {
    cv::Scalar color_contours{255, 255, 255}; 
    drawContours(mask, contour_list, i, color_contours, -1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
  }

  cv::Mat masked_image;

  image.copyTo(masked_image, mask);


  int const red_threshold = 200;
  int const green_threshold = 200;
  int const blue_threshold = 200;

  cv::Mat lane_markers;
  cv::inRange(masked_image, cv::Scalar(blue_threshold, green_threshold, red_threshold), cv::Scalar(255, 255, 255), lane_markers);

  cv::Mat output = image.clone();

  output.setTo(cv::Scalar(0,0,255), lane_markers);

  cv::imshow("Display Image", image);
  cv::imshow("Output Image", output);
  cv::waitKey(0);

  return 0;
}