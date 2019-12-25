#pragma once
#include <string>
#include <opencv2/opencv.hpp>

std::string type2str(int type);

// helper function to see if a value is within defined limits
template<class T>
bool in_bounds(T val, T min, T max){
  return val >= min && val <= max;}


template <class T>
T AbsMax (cv::Mat img){
  T min_val = 0;
  T max_val = 0;
  cv::minMaxIdx(img, &min_val, &max_val);
  return std::max(abs(min_val), abs(max_val));}

std::vector<std::string> get_files(const std::string& directory);

void ThresholdImage(cv::Mat input, cv::Mat& output, double min_val, double max_val);
