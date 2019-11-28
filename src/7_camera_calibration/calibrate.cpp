#include "cv_common.hpp"
#include "calibrate.hpp"
#include "fmt/core.h" 
#include "fmt/format.h"

void extractPointsFromImage(cv::Mat img, cv::Size board_size, CameraPoints* cp){
  cv::Mat gray_img;
  cv::cvtColor(img, gray_img, CV_BGR2GRAY);
  
  static std::vector<cv::Point2f> corners;
  corners.clear();

  bool pattern_found = cv::findChessboardCorners(img, board_size, corners);
  if(pattern_found){
    fmt::print("pattern found\n");
    cv::cornerSubPix(gray_img, corners, cv::Size{5,5}, cv::Size{-1,-1}, 
      cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
    cv::drawChessboardCorners(gray_img, board_size, corners, pattern_found);}
  
  if (pattern_found){
    std::vector<cv::Point3f> obj_pts;
    for(int y = 0; y < board_size.height; ++y){
      for(int x = 0; x < board_size.width; ++x){
        obj_pts.emplace_back( (float) x, (float) y,0 );}}

    cp->object_points.push_back(obj_pts);
    cp->image_points.push_back(corners);}}