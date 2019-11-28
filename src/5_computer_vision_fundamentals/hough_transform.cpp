#include "fmt/core.h" 
#include <string>
#include <opencv2/opencv.hpp>
#include "cv_common.hpp"

static double const kPi = 3.14159265359;

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

  cv::imshow("Input", image);

  cv::Mat gray_image; 
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  fmt::print("image size: {}x{} type: {}\n", gray_image.size[1], gray_image.size[0], type2str(gray_image.type()));
  
  // Define a kernel size for Gaussian smoothing / blurring
  // Note: this step is optional as cv::Canny() applies a 5x5 Gaussian internally
  cv::Size kernel_size {5, 5};
  cv::Mat blur_gray; 
  cv::GaussianBlur(gray_image, blur_gray, kernel_size, 0);

  // Define parameters for Canny and run it
  int low_threshold = 50;
  int high_threshold = 150;
  cv::Mat canny; 
  cv::Canny(blur_gray, canny, low_threshold, high_threshold);
  cv::imshow("Canny edges", canny);

  std::vector<std::vector<cv::Point>> vertices {{{0,y_size},{450,290},{490,290},{x_size,y_size}}};

  cv::Mat mask = cv::Mat::zeros(canny.size(),CV_8UC1);

  cv::fillPoly(mask, vertices, cv::Scalar(255));

  cv::Mat masked_canny;
  canny.copyTo(masked_canny, mask);
  cv::imshow("Masked Canny Image", masked_canny);

  // Define the Hough transform parameters
  // Make a blank the same size as our image to draw on
  int rho = 2;
  double theta = kPi/180;
  int threshold = 15;
  int min_line_length = 40;
  int max_line_gap = 20;

  cv::Mat masked_canny_color;
  cv::cvtColor(masked_canny, masked_canny_color, CV_GRAY2BGR);

  //create blank bgr image to draw lines on
  cv::Mat color_lines = cv::Mat::zeros(masked_canny_color.size(), masked_canny_color.type()); 
  
  // Run Hough on edge detected image
  std::vector<cv::Vec4i> hough_lines;
  cv::HoughLinesP(masked_canny, hough_lines, rho, theta, threshold, min_line_length, max_line_gap);
  
  for (const auto& line : hough_lines){
    cv::line(color_lines, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,0,255), 3, CV_AA);}

// # Draw the lines on the edge image
  cv::Mat combo;
  cv::addWeighted(color_lines, 0.8, masked_canny_color, 1, 0, combo); 

  cv::imshow("hough", combo);
  cv::waitKey(0);
}

