#include "fmt/core.h" 
#include <string>
#include <opencv2/opencv.hpp>
#include "cv_helpers.hpp"

// ###### MODIFY THESE VARIABLES TO MAKE YOUR COLOR SELECTION
static int const red_threshold = 200;
static int const green_threshold = 200;
static int const blue_threshold = 200;
// ######

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

  cv::Mat output;
  cv::inRange(image, cv::Scalar(blue_threshold, green_threshold, red_threshold), cv::Scalar(255, 255, 255), output);

  cv::imshow("Display Image", image);
  cv::imshow("Output Image", output);
  cv::waitKey(0);
  return 0;
}