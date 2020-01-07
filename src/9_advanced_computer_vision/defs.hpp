#pragma once
#include <vector>
#include "cv_common.hpp"

struct Params{
  cv::Mat image;
  std::string image_file_path;
  std::vector<std::string> image_files;
  std::string current_file;
  cv::Mat K;
  cv::Mat D;
  cv::Mat M;
  cv::Mat M_inv;
  uint8_t min_sobel = 20; // 100
  uint8_t max_sobel = 100; // 255
  uint8_t min_s = 175;
  uint8_t max_s = 255;
  uint8_t min_l = 200;
  uint8_t max_l = 255;
  uint8_t nwindows = 12; //> number of sliding windows
  uint16_t margin = 250; //> Set the width of the windows +/- margin
  uint16_t minpix = 100; //> Set minimum number of pixels found to recenter window
  float xm_per_pix = 0;
  float ym_per_pix = 0;
  bool demo = false; };