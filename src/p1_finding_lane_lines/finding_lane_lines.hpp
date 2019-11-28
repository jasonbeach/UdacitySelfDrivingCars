#pragma once
#include "cv_defs.hpp"

// this is the main function that processes a frame.  Yes it should be broken up
// into smaller functions.
void FindLanes(const cv::Mat& input, cv::Mat& output, const FindLaneParams& p);

// This function doesn ransac on a set of given lines. It could probably use
// some optimization for performance as this is likely the most computationally
// expensive portion of the algorithm 
ModelSet do_ransac(const cv_line_list& input_lines, bool positive_slope);

template< class T>
T alpha_filter(const T& current_value, const T& new_value, float alpha){
  return alpha * (new_value) + (1-alpha) * current_value;}

// updates a Mat with the ransac output
void plot_model(int y_size, const cv::Mat1d& model, const cv::Scalar color, 
  cv::Mat* image, std::vector<cv::Point> * points );

// helper function to see if a value is within defined limits
bool in_bounds(int val, int min, int max);

// these next several functions are used by the trackbars to adjust that
// trackbars respective value
void adjust_canny_low( int count, void* param);

void adjust_canny_high( int count, void* param);

void adjust_hough_threshold( int count, void* param);

void adjust_hough_min_length( int count, void* param);

void adjust_hough_max_gap( int count, void* param);

void adjust_line_min( int count, void* param);

void adjust_line_max( int count, void* param);

void adjust_top_width( int count, void* param);

void adjust_top_offset( int count, void* param);

void toggle_image( int state, void* param);

void adjust_frame_speed (int ms_delay, void* param);

// create the window with all the track bars
void setup_windows(FindLaneParams* find_lane_params);

