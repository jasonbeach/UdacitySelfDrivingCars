#pragma once
#include <vector>
#include "cv_common.hpp"

struct Params{
  std::string video_filename;
  cv::Mat K;
  cv::Mat D;
  cv::Mat M;
  uint8_t min_sobel;
  uint8_t max_sobel;
  uint8_t min_s;
  uint8_t max_s;
  uint8_t min_l;
  uint8_t max_l;
  uint8_t nwindows; //> number of sliding windows
  uint16_t margin; //> Set the width of the windows +/- margin
  uint16_t minpix; //> Set minimum number of pixels found to recenter window
  float xm_per_pix;
  float ym_per_pix;
  bool loop_video;
  bool show_trackbars;
  bool record_video;
  bool initialized = false;  };
  