#include "fmt/core.h" 
#include "fmt/format.h"
#include <string>
#include "CLI/CLI.hpp"
#include <iostream>
#include "cv_common.hpp"
#include "calibrate.hpp"


int main(int argc, char** argv ){
  
  CLI::App app{"checkerboard"};

  std::string file_path = "./test_images/project2/calibration_test.png";

  app.add_option("-f,--file", file_path, "path to files");
  CLI11_PARSE(app, argc, argv);
  
  cv::Mat image = cv::imread( file_path, cv::IMREAD_COLOR );

  if ( !image.data ){
    fmt::print("No image data \n");
    return -1;}

  auto y_size = image.size[0];
  auto x_size = image.size[1];
  fmt::print("image size: {}x{} type: {}\n", x_size, y_size, type2str(image.type()));

  cv::Mat gray_image; 
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  const int nx = 8;
  const int ny = 6;


  CameraPoints cp;

  cv::Size size{nx, ny};

  cv::Mat K, D;
  std::vector<cv::Mat> rvecs, tvecs;
  
  extractPointsFromImage(image, size, &cp, true);

  fmt::print("image pts size: {} x 2, object pts size: {} x 3\n", 
    cp.image_points.size(), cp.object_points.size());
  cv::calibrateCamera(cp.object_points, cp.image_points, image.size(), K, D, rvecs, tvecs);

  auto re = computeReprojectionErrors(cp.object_points, cp.image_points, rvecs, tvecs, K, D);

  fmt::print("reprojection error: {:.2f}\n", re);

  cv::Mat udist_image;
  cv::undistort(image, udist_image, K, D);

  std::cout << "k matrix: \n" << K << std::endl;
   
  cv::imshow("Image", udist_image);
  cv::waitKey(0);
  return 0;
}