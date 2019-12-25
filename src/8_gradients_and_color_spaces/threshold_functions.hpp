#pragma once

#include "cv_common.hpp"

struct Params{
  cv::Mat image;
  std::vector<std::string> image_files;
  cv::Mat K;
  cv::Mat D;
  std::string orientation = "x";
  uint8_t min_thresh = 20; // 100
  uint8_t max_thresh = 100; // 255
  uint8_t min_mag = 30;
  uint8_t max_mag = 100;
  uint8_t mag_sobel_kernel = 9;
  double min_angle = .7;
  double max_angle = 1.3;
  uint8_t dir_sobel_kernel = 15;
  uint8_t min_s = 140;
  uint8_t max_s = 255;
  uint8_t min_l = 200;
  uint8_t max_l = 255;};

void adjust_sobel_min( int count, void* param);

void adjust_sobel_max( int count, void* param);

cv::Mat abs_sobel_thresh(cv::Mat img, std::string orient, uint8_t thresh_min, uint8_t thresh_max);

void adjust_min_mag( int count, void* param);

void adjust_max_mag( int count, void* param);

cv::Mat mag_threshold(cv::Mat img, uint8_t sobel_kernel, double min_thresh, double max_thresh);

void adjust_min_angle( int count, void* param);

void adjust_max_angle( int count, void* param);

cv::Mat dir_threshold(cv::Mat img, uint8_t sobel_kernel, double min_thresh, double max_thresh);

void display_image(const Params& p);

void adjust_s_min( int count, void* param);

void adjust_s_max( int count, void* param);

void adjust_l_min( int count, void* param);

void adjust_l_max( int count, void* param);

void change_image( int count, void* param);

cv::Mat thresh_color(cv::Mat img_bgr, uint8_t channel, uint8_t min_thresh, uint8_t max_thresh);