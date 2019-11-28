#include "fmt/core.h" 
#include <string>
#include <opencv2/opencv.hpp>
#include "cv_common.hpp"


int main(int argc, char** argv ){
  std::string filename;
  if ( argc != 2 ){
    filename = "exit-ramp.jpg";}
  else{
    filename = std::string(argv[1]);}
  
  cv::Mat image = cv::imread( filename, cv::IMREAD_COLOR );
  
  if ( !image.data ){
    fmt::print("No image data \n");
    return -1;}

  auto y_size = image.size[0];
  auto x_size = image.size[1];
  fmt::print("image size: {}x{} type: {}\n", x_size, y_size, type2str(image.type()));

  cv::Mat gray_image; 
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  // Define a kernel size for Gaussian smoothing / blurring
  // Note: this step is optional as cv::Canny() applies a 5x5 Gaussian internally
  cv::Size kernel_size {5, 5};
  cv::Mat blur_gray; 
  cv::GaussianBlur(gray_image, blur_gray, kernel_size, 0);

  // Define parameters for Canny and run it
  int low_threshold = 50;
  int high_threshold = 150;
  cv::Mat edges; 
  cv::Canny(blur_gray, edges, low_threshold, high_threshold);

  cv::imshow("Input", image);
  cv::imshow("blurred", blur_gray);
  cv::imshow("Output Image", edges);
  cv::waitKey(0);
}

