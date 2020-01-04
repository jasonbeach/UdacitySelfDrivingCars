#pragma once
#include "cv_common.hpp"

struct AdvancedLaneFinderParams{
  std::string trackbar_window;
  cv::Mat K;
  cv::Mat D;
  cv::Mat M;
  float xm_per_pix;
  float ym_per_pix;
  uint16_t margin; //> Set the width of the windows +/- margin
  uint16_t minpix; //> Set minimum number of pixels found to recenter window
  uint8_t nwindows; //> number of sliding windows
  uint8_t min_sobel;
  uint8_t max_sobel;
  uint8_t min_s;
  uint8_t max_s;
  uint8_t min_l;
  uint8_t max_l;
  bool show_trackbars;
  int show_warped = false;
  bool initialized = false;  };

struct LaneLineModel{
  float a = 0;
  float b = 0;
  float c = 0;
  bool valid = false;
  
  float calc_x(float y) const{
    return a*SQ(y) + b*y + c; } };
