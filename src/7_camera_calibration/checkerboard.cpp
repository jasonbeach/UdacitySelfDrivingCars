#include "fmt/core.h" 
#include "fmt/format.h"
#include <string>
#include "CLI/CLI.hpp"
#include <iostream>
#include "cv_common.hpp"
#include "calibrate.hpp"


int main(int argc, char** argv ){
  
  CLI::App app{"checkerboard"};

  std::string file_path = "./test_images/calibration_test.png";

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
  
  extractPointsFromImage(image, size, &cp);

  fmt::print("image pts size: {} x 2, object pts size: {} x 3\n", 
    cp.image_points.size(), cp.object_points.size());
  cv::calibrateCamera(cp.object_points, cp.image_points, size, K, D, rvecs, tvecs);

  cv::Mat udist_image;
  cv::undistort(image, udist_image, K, D);

  std::cout << "k matrix: \n" << K << std::endl;
   
  cv::imshow("Image", udist_image);
  cv::waitKey(0);
  return 0;
}


/*
def corners_unwarp(img, nx, ny, mtx, dist):
    # Pass in your image into this function
    # Write code to do the following steps
    # 1) Undistort using mtx and dist
    udist_img = cv2.undistort(img, mtx, dist)
    # 2) Convert to grayscale
    udist_gray = cv2.cvtColor(udist_img, cv2.COLOR_BGR2GRAY)
    # 3) Find the chessboard corners
    pattern_found, corners = cv2.findChessboardCorners(udist_img, (nx, ny))
    
    img_size = (udist_gray.shape[1], udist_gray.shape[0])
    warped = img
    M = None
    if pattern_found == True:
    # 4) If corners found: 
            # a) draw corners
            # b) define 4 source points src = np.float32([[,],[,],[,],[,]])
                 #Note: you could pick any four of the detected corners 
                 # as long as those four corners define a rectangle
                 #One especially smart way to do this would be to use four well-chosen
                 # corners that were automatically detected during the undistortion steps
                 #We recommend using the automatic detection of corners in your code
        src = np.float32([corners[0], corners[nx-1], corners[-1], corners[-nx]])
            # c) define 4 destination points dst = np.float32([[,],[,],[,],[,]])
        offset = 100
        dst = np.float32([[offset, offset], [img_size[0]-offset, offset], 
                                     [img_size[0]-offset, img_size[1]-offset], 
                                     [offset, img_size[1]-offset]])
            # d) use cv2.getPerspectiveTransform() to get M, the transform matrix
        M = cv2.getPerspectiveTransform(src, dst)
        print("M: \n{}".format(M))
        YNMMV
*/