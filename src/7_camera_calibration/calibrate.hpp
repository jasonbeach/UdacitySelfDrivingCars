#include "cv_common.hpp"
#include <vector>

struct CameraPoints{
  std::vector<std::vector<cv::Point3f> > object_points;
  std::vector<std::vector<cv::Point2f> > image_points;};

void extractPointsFromImage(cv::Mat img, cv::Size board_size, CameraPoints* cp);
