#include "cv_common.hpp"
#include "calibrate.hpp"
#include "fmt/core.h" 
#include "fmt/format.h"

void extractPointsFromImage(cv::Mat img, cv::Size board_size, CameraPoints* cp, bool show_images){
  cv::Mat gray_img;
  cv::cvtColor(img, gray_img, CV_BGR2GRAY);
  
  static std::vector<cv::Point2f> corners;
  corners.clear();

  bool pattern_found = cv::findChessboardCorners(img, board_size, corners);
  if(pattern_found){
    fmt::print("pattern found\n");
    cv::cornerSubPix(gray_img, corners, cv::Size{5,5}, cv::Size{-1,-1}, 
      cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));}

  if(show_images){    
    cv::drawChessboardCorners(img, board_size, corners, pattern_found);
    cv::imshow("Image", img);
    cv::waitKey(0);}
  
  if (pattern_found){
    std::vector<cv::Point3f> obj_pts;
    for(int y = 0; y < board_size.height; ++y){
      for(int x = 0; x < board_size.width; ++x){
        obj_pts.emplace_back( (float) x, (float) y,0 );}}

    cp->object_points.push_back(obj_pts);
    cp->image_points.push_back(corners);}}
  
double computeReprojectionErrors(
  const std::vector< std::vector< cv::Point3f > >& objectPoints,
  const std::vector< std::vector< cv::Point2f > >& imagePoints,
  const std::vector< cv::Mat >& rvecs, const std::vector< cv::Mat >& tvecs,
  const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs) {
  
  std::vector< cv::Point2f > imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  std::vector< float > perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) {
    cv::projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}