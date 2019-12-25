#include "threshold_functions.hpp"
#include "fmt/core.h" 


void adjust_sobel_min( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  
  if(in_bounds<uint8_t>(count, 0, p->max_thresh)){
    p->min_thresh = count;
    display_image(*p);}}

void adjust_sobel_max( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  
  if(in_bounds<uint8_t>(count, p->min_thresh, 255)){
    p->max_thresh = count;
    display_image(*p);}}

cv::Mat abs_sobel_thresh(cv::Mat img, std::string orient, uint8_t thresh_min, uint8_t thresh_max){

  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);

  cv::Mat sobel_out;
  if(orient == "x"){
    cv::Sobel(img_gray, sobel_out, CV_64F, 1, 0);}
  else if(orient == "y"){
    cv::Sobel(img_gray, sobel_out, CV_64F, 0, 1);}
  else if(orient == "z"){
    cv::Laplacian(img_gray, sobel_out, CV_64F,15);}
  else{
    return {}; }

  cv::Mat abs_img, abs_img_thres;

  double abs_max = AbsMax<double>(sobel_out);
  //fmt::print("max: {}\n", abs_max);

  double scale = 255.0 / abs_max;

  cv::convertScaleAbs(sobel_out, abs_img, scale);
  
  ThresholdImage(abs_img, abs_img_thres, thresh_min, thresh_max);

  return abs_img_thres;}

void adjust_min_mag( int count, void* param){
  Params* p = static_cast<Params*>(param);  
    
  if(in_bounds<uint8_t>(count, 0, p->max_mag)){
    p->min_mag = count;
    display_image(*p);}}

void adjust_max_mag( int count, void* param){
  Params* p = static_cast<Params*>(param);  

  if(in_bounds<uint8_t>(count, p->min_mag, 255)){
    p->max_mag = count;
    display_image(*p);}}

cv::Mat mag_threshold(cv::Mat img, uint8_t sobel_kernel, double min_thresh, double max_thresh){

  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);

  cv::Mat sobel_x, sobel_y, grad_out;
  
  cv::Sobel(img_gray, sobel_x, CV_64F, 1, 0, sobel_kernel);
  cv::Sobel(img_gray, sobel_y, CV_64F, 0, 1, sobel_kernel);

  sobel_x = cv::abs(sobel_x);
  sobel_y = cv::abs(sobel_y);

  cv::magnitude(sobel_x, sobel_y, grad_out);

  cv::Mat abs_img;

  double max_val = AbsMax<double>(grad_out);
  double scale = 255.0 / max_val;

  cv::Mat out;

  cv::convertScaleAbs(grad_out, out, scale);
  ThresholdImage(out, out, min_thresh, max_thresh);  

  return out; }

void adjust_min_angle( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  double new_min = (double) count / 100.0;
  
  if(in_bounds(new_min, 0.0, p->max_angle)){
    p->min_angle = new_min;
    display_image(*p);}}

void adjust_max_angle( int count, void* param){
  Params* p = static_cast<Params*>(param);  
  double new_max = (double) count / 100.0;
  
  if(in_bounds(new_max, p->min_angle, 2.0 * M_PI)){
    p->max_angle = new_max;
    display_image(*p);}}

cv::Mat dir_threshold(cv::Mat img, uint8_t sobel_kernel, double min_thresh, double max_thresh){

  cv::Mat img_gray;
  if(img.channels() > 1){
    cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);}
  else{
    img_gray = img;}

  cv::Mat sobel_x, sobel_y, grad_out;
  
  cv::Sobel(img_gray, sobel_x, CV_64F, 1, 0, sobel_kernel);
  cv::Sobel(img_gray, sobel_y, CV_64F, 0, 1, sobel_kernel);

  sobel_x = cv::abs(sobel_x);
  sobel_y = cv::abs(sobel_y);

  cv::phase(sobel_x, sobel_y, grad_out);

  cv::Mat abs_img;

  double max_val = AbsMax<double>(grad_out);
  double scale = 255.0 / max_val;

  cv::Mat out;

  ThresholdImage(grad_out, out, min_thresh, max_thresh);
  cv::convertScaleAbs(out, out, scale);

  return out; }

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
