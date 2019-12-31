#include "threshold_functions.hpp"
#include "fmt/core.h" 

void adjust_sobel_min( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  
  if(in_bounds<uint8_t>(count, 0, p->max_sobel)){
    p->min_sobel = count;
    display_image(*p);}}

void adjust_sobel_max( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  
  if(in_bounds<uint8_t>(count, p->min_sobel, 255)){
    p->max_sobel = count;
    display_image(*p);}}

cv::Mat abs_sobel_thresh(cv::Mat img, uint8_t thresh_min, uint8_t thresh_max){

  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);

  cv::Mat sobel_out;
  cv::Sobel(img_gray, sobel_out, CV_64F, 1, 0);
  
  cv::Mat abs_img, abs_img_thres;

  double abs_max = AbsMax<double>(sobel_out);

  double scale = 255.0 / abs_max;

  cv::convertScaleAbs(sobel_out, abs_img, scale);
  
  ThresholdImage(abs_img, abs_img_thres, thresh_min, thresh_max);

  return abs_img_thres;}

void adjust_s_min( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  
  if(in_bounds<uint8_t>(count, 0, p->max_s)){
    p->min_s = count;
    display_image(*p);}}

void adjust_s_max( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  
  if(in_bounds<uint8_t>(count, p->min_s, 255)){
    p->max_s = count;
    display_image(*p);}}

void adjust_l_min( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  
  if(in_bounds<uint8_t>(count, 0, p->max_l)){
    p->min_l = count;
    display_image(*p);}}

void adjust_l_max( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  
  if(in_bounds<uint8_t>(count, p->min_l, 255)){
    p->max_l = count;
    display_image(*p);}}

void change_image( int count, void* param){
  Params* p = static_cast<Params*>(param);

  cv::Mat image = cv::imread( p->image_files.at(count), cv::IMREAD_COLOR );
  cv::undistort(image, p->image, p->K, p->D);
  display_image(*p);}

cv::Mat thresh_color(cv::Mat img_bgr, uint8_t channel, uint8_t min_thresh, uint8_t max_thresh){

  cv::Mat img_hls, img_chan;
  cv::cvtColor(img_bgr, img_hls, CV_BGR2HLS);

  cv::extractChannel(img_hls, img_chan, channel);

  ThresholdImage(img_chan, img_chan, min_thresh, max_thresh);

  return img_chan; }
