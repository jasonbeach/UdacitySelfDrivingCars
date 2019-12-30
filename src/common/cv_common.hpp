#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include "fmt/core.h"

std::string type2str(int type);

// helper function to see if a value is within defined limits
template<class T>
bool in_bounds(T val, T min, T max){
  return val >= min && val <= max;}

// This is a very badly implemented helper function that to allows a cv::Mat 
// object to work with libfmt. In it's current form it *only* works properly 
// a  

template <>
struct fmt::formatter<cv::Mat> {
  constexpr auto parse(format_parse_context& ctx) { 
    return ctx.begin(); }

  template <typename FormatContext>
  auto format(const cv::Mat& m, FormatContext& ctx) {

    ctx.out() = format_to(ctx.out(), "size: {}x{} type: {}", m.rows, m.cols, type2str(m.type()));

    if(m.type() != 6 || !m.isContinuous()){
      return ctx.out(); }
    
    ctx.out() = format_to(ctx.out(), "\n");
    
    for (int row = 0; row < m.rows; ++row){
      for (int col = 0; col < m.cols; ++ col){
        if(col == 0){
          ctx.out() = format_to(ctx.out(), "  {:.2f}", m.at<double>(row, col));}
        else{
          ctx.out() = format_to(ctx.out(), ", {:.2f}", m.at<double>(row, col));} }
      ctx.out() = format_to(ctx.out(), "\n"); }

    return ctx.out();  } };


template <class T>
T AbsMax (cv::Mat img){
  T min_val = 0;
  T max_val = 0;
  cv::minMaxIdx(img, &min_val, &max_val);
  return std::max(abs(min_val), abs(max_val));}

std::vector<std::string> get_files(const std::string& directory);

void ThresholdImage(cv::Mat input, cv::Mat& output, double min_val, double max_val);
