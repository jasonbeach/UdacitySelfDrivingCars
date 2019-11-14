#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include "fmt/core.h" 

static double constexpr kPi = 3.14159265359;
static double constexpr kD2R = kPi / 180.0;
static double constexpr kR2D = 180.0 / kPi;

//The main struct that holds all the params with default values
struct FindLaneParams{

  int canny_blur_kernal_size = 5; // kernel for the gaussian blur
  int canny_low = 50; // canny low threshold
  int canny_high = 150; // canny high threshold
  int mask_top_width = 70; // width of the top of the mask trapezoid
  int mask_top_offset = 0; // offset from center of the top os the mask trapezoid
  double mask_height = .6; // height of the mask trapezoid as a percent of the image height
  int hough_rho = 2; // rho for hough transform
  double hough_theta = 1.0 * kD2R; // theta for hough transform
  int hough_threshold = 15; //min number of intersections for hough transform
  int hough_min_line_length = 30; // min line length for hough transform
  int hough_max_line_gap = 40; // max gap width for hough transform
  double line_min_angle = 50 * kD2R; // lower limit for valid lines from hough transform
  double line_max_angle = 60 * kD2R; // upper limit for valid lines from hough transform
  bool show_canny = false; // show the annotated color image or annotated hough lines
  bool loop_video = false; // whether or not to loop the video continuously
  int frame_delay_ms = 40; // added delay between each frame
  bool show_trackbars = false; //whether or not to show window with adjustment trackbars

  // print the parameters as a string
  std::string str(){
    return fmt::format(
      "canny blur kernel size: {}\n"
      "canny low: {}\n"
      "canny high: {}\n"
      "mask top width: {}\n"
      "mask top offset: {}\n"
      "mask height: {}\n"
      "hough rho: {}\n"
      "hough theta: {}\n"
      "hough threshold: {}\n"
      "hough min line length: {}\n"
      "hough max line gap: {}\n"
      "line min angle: {}\n"
      "line max angle: {}\n"
      "frame_delay_ms: {}\n",
      canny_blur_kernal_size, canny_low, canny_high, mask_top_width, mask_top_offset,
      mask_height, hough_rho, hough_theta * kR2D, hough_threshold, hough_min_line_length,
      hough_max_line_gap, line_min_angle * kR2D, line_max_angle* kR2D, frame_delay_ms);}};

using cv_line = cv::Vec4i;
using cv_line_list = std::vector<cv_line>;

// candidate model set for ransac
struct ModelSet{
  cv::Mat1d model {};
  std::vector<cv::Point> inliers{}; // decompose inliers into points
  std::vector<cv::Vec4i> outliers{}; // keep outliers as lines

  // convienence function to decompose line into two points
  void add_inlier(const cv::Vec4i& inlier){
    inliers.emplace_back(inlier[0], inlier[1]);
    inliers.emplace_back(inlier[2], inlier[3]);}};