#pragma once
#include "defs.hpp"

void adjust_sobel_min( int count, void* param);

void adjust_sobel_max( int count, void* param);

cv::Mat abs_sobel_thresh(cv::Mat img, uint8_t thresh_min, uint8_t thresh_max);

void display_image(const Params& p);

void adjust_s_min( int count, void* param);

void adjust_s_max( int count, void* param);

void adjust_l_min( int count, void* param);

void adjust_l_max( int count, void* param);

void change_image( int count, void* param);

cv::Mat thresh_color(cv::Mat img_bgr, uint8_t channel, uint8_t min_thresh, uint8_t max_thresh);