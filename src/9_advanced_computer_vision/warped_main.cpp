#include <iostream>
#include <string>
#include <chrono>
#include <random>

#include "threshold_functions.hpp"
#include "fmt/core.h"
#include "CLI/CLI.hpp"
#include "yaml_helpers.hpp"

int get_max_row(const cv::Mat& img){
  cv::Mat summed;
  cv::reduce(img, summed, 0, CV_REDUCE_SUM, CV_32S);

  cv::Point max_loc;

  cv::minMaxLoc(summed, nullptr, nullptr, nullptr, &max_loc);
  return max_loc.x;}

cv::Mat nonzero_test(const cv::Mat& img){
  cv::threshold(img, img, 0, 255, CV_THRESH_BINARY);
  fmt::print("img size: {}x{} type: {}\n", img.cols, img.rows, type2str(img.type()));
  cv::Mat empty = cv::Mat::zeros(img.size(), CV_8U);
  cv::Mat nonzero_channel = cv::Mat::zeros(img.size(), CV_8U);
  fmt::print("empty Mat:  rows: {} cols: {} type: {}\n", empty.rows, empty.cols, type2str(empty.type()));
  fmt::print("nonzero channel Mat:  rows: {} cols: {} type: {}\n", nonzero_channel.rows, nonzero_channel.cols, type2str(nonzero_channel.type()));

  //cv::Mat nonzero;
  std::vector<cv::Point> locations;
  cv::findNonZero(img, locations);
  //fmt::print("nonzero Mat:  rows: {} cols: {} type: {}\n", nonzero.rows, nonzero.cols, type2str(nonzero.type()));
  fmt::print("locations: size: {}\n ", locations.size());  
  
  for(const auto& pt : locations){
    nonzero_channel.at<uint8_t>(pt.y, pt.x) = 255;
  }

  // for(int i = 0; i < nonzero.rows; ++i){
  //   cv::Point pt = nonzero.at<cv::Point>(i);
  //   nonzero_channel.at<uint8_t>(pt.y, pt.x) = 255;
  // }

  cv::Mat arry[3] = {img, nonzero_channel, empty};

  cv::Mat merged;
  cv::merge(arry, 3, merged);
  return merged;

}

cv::Mat find_lane_line(const cv::Mat& warped_image, const cv::Rect& roi, const Params& p){

  cv::Mat mask = cv::Mat::zeros(warped_image.size(), CV_8U); // all 0

  mask(roi) = 255;

  cv::Mat masked_warped;
  warped_image.copyTo(masked_warped, mask);

  int lane_line_center = get_max_row(masked_warped);

  //fmt::print("min: {} loc: {} max: {} loc: {}\n", min_val, min_loc.x, max_val, max_loc.x);
  //cv::line(warped_left, {max_loc.x, warped_left.rows/2},{max_loc.x, warped_left.rows}, {255},2 );

  int rows = warped_image.rows;

  int window_height = rows / p.nwindows;

  int current_x = lane_line_center - p.margin / 2; //upper left corner of rect
  int current_y = rows - window_height;            // upper left corner of rect
  
  cv::Mat empty = cv::Mat::zeros(warped_image.size(), CV_8U);
  cv::Mat nonzero_channel = cv::Mat::zeros(warped_image.size(), CV_8U);

  while(current_y >= 0 ){
    if (current_x < 0){
      current_x = 0; }
      
    if (current_x > warped_image.cols - p.margin){
      current_x = warped_image.cols - p.margin; }
    
    //fmt::print("x: {} y: {} margin: {} win height: {}\n", current_x, current_y, p.margin, window_height);
    cv::Rect window_roi = cv::Rect(current_x, current_y, p.margin, window_height);
    cv::Mat image_window = warped_image(window_roi);

    cv::Mat nonzero;
    cv::findNonZero(image_window, nonzero);
  
    for(int i = 0; i < nonzero.rows; ++i){
      cv::Point pt = nonzero.at<cv::Point>(i);
      nonzero_channel.at<uint8_t>(pt.y + current_y, pt.x + current_x) = 255;
    }

    if (nonzero.rows > p.minpix){
      cv::Mat x_pts;
      cv::extractChannel(nonzero, x_pts, 0);

      cv::Mat mean;
      x_pts.convertTo(mean, CV_32F);
      cv::reduce(mean, mean, 0, CV_REDUCE_AVG, CV_32F);

      current_x +=  mean.at<float>(0,0) - p.margin/2;}


    current_y -= window_height; 
    cv::rectangle(empty, window_roi, {255});} //draw  current row

  cv::Mat arry[3] = {warped_image, nonzero_channel, empty};

  cv::Mat merged;
  cv::merge(arry, 3, merged);

  return merged;
}

void display_image(const Params& p){

  auto t1 = std::chrono::high_resolution_clock::now();
  cv::Mat abs_img = abs_sobel_thresh(p.image, p.orientation, p.min_thresh, p.max_thresh);
  //cv::Mat mag_img = mag_threshold(p.image, p.mag_sobel_kernel, p.min_mag, p.max_mag);
  //cv::Mat dir_img = dir_threshold(mag_img, p.dir_sobel_kernel, p.min_angle, p.max_angle);
  //cv::Mat sobel_chan;

  //cv::bitwise_and(abs_img, mag_img, sobel_chan);
  //cv::bitwise_and(sobel_chan, dir_img, sobel_chan);

  cv::Mat s_chan = thresh_color(p.image, 2, p.min_s, p.max_s);
  cv::Mat l_chan = thresh_color(p.image, 1, p.min_l, p.max_l);
 
  cv::Mat color_chan;
  cv::Mat merged;
  
  cv::bitwise_or(s_chan, l_chan, color_chan);
  cv::bitwise_or(color_chan, abs_img, merged);

  //cv::Mat empty = cv::Mat::zeros(s_chan.size(), CV_8UC1);
  
  //cv::Mat arry[3] = {color_chan, mag_img /*sobel_chan*/, empty};

  //cv::merge(arry, 3, merged);

  cv::Mat warped_img;
  cv::warpPerspective(merged, warped_img, p.M, merged.size());
  cv::threshold(warped_img, warped_img, 128, 255, CV_THRESH_BINARY);

  //cv::Mat combined = nonzero_test(warped_img);
  cv::Mat left_line = find_lane_line(warped_img, cv::Rect(0,warped_img.rows/2,warped_img.cols/2, warped_img.rows/2), p);
  cv::Mat right_line = find_lane_line(warped_img, cv::Rect(warped_img.cols/2, warped_img.rows/2, warped_img.cols/2, warped_img.rows/2), p);

 
  auto t2 = std::chrono::high_resolution_clock::now();
  auto process_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
  fmt::print("processing time: {} ms\n", process_time_ms);

  auto combined = .5 * left_line + .5 * right_line;

  cv::imshow("image", combined); 
  cv::imshow("unwarped image", merged);

  } 
  

int main(int argc, char** argv ){

  Params p; 
  CLI::App app{"color thresholds"};
  
  std::string config_yaml = "config.yaml";
  app.add_option("-f, --file", config_yaml, "path to yaml file");
  
  CLI11_PARSE(app, argc, argv);
  
  YAML::Node config = YAML::LoadFile(config_yaml);

  std::string image_file_path;
  if(config["image_file_path"]){
    image_file_path = config["image_file_path"].as<std::string>(); 
    p.image_files = get_files(image_file_path);}


  if(config["K"]){
    p.K = config["K"].as<cv::Mat>(); }

  if(config["D"]){
    p.D = config["D"].as<cv::Mat>();}

  if(config["M"]){
    p.M = config["M"].as<cv::Mat>();}

  if(p.image_files.empty()){
    fmt::print("No files given in: {}\n", image_file_path);
    return -1;}

  cv::Mat image = cv::imread( p.image_files.at(0), cv::IMREAD_COLOR );

  if ( !image.data ){
    fmt::print("opening {} failed\n", p.image_files.at(0));
    exit(-1);}

  fmt::print("image size: {}x{} type: {}\n", image.cols, image.rows, type2str(image.type()));
  
  
  cv::undistort(image, p.image, p.K, p.D);
  
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("adjustments", CV_WINDOW_AUTOSIZE);

  cv::createTrackbar("sobel min","adjustments", nullptr, 255, &adjust_sobel_min, &p);
  cv::setTrackbarPos("sobel min","adjustments", p.min_thresh);
 
  cv::createTrackbar("sobel max","adjustments", nullptr, 255, &adjust_sobel_max, &p);
  cv::setTrackbarPos("sobel max","adjustments",p.max_thresh);

  cv::createTrackbar("min mag","adjustments", nullptr, 255, &adjust_min_mag, &p);
  cv::setTrackbarPos("min mag","adjustments", p.min_mag);
 
  cv::createTrackbar("max mag","adjustments", nullptr, 255, &adjust_max_mag, &p);
  cv::setTrackbarPos("max mag","adjustments",p.max_mag);

  cv::createTrackbar("min angle","adjustments", nullptr, 157, &adjust_min_angle, &p);
  cv::setTrackbarPos("min angle","adjustments", p.min_angle * 100.0);
 
  cv::createTrackbar("max angle","adjustments", nullptr, 157, &adjust_max_angle, &p);
  cv::setTrackbarPos("max angle","adjustments",p.max_angle * 100);

  cv::createTrackbar("min s","adjustments", nullptr, 255, &adjust_s_min, &p);
  cv::setTrackbarPos("min s","adjustments", p.min_s);
 
  cv::createTrackbar("max s","adjustments", nullptr, 255, &adjust_s_max, &p);
  cv::setTrackbarPos("max s","adjustments",p.max_s);

  cv::createTrackbar("min l","adjustments", nullptr, 255, &adjust_l_min, &p);
  cv::setTrackbarPos("min l","adjustments", p.min_l);
 
  cv::createTrackbar("max l","adjustments", nullptr, 255, &adjust_l_max, &p);
  cv::setTrackbarPos("max l","adjustments",p.max_l);

  cv::createTrackbar("image", "adjustments", nullptr, p.image_files.size() - 1, &change_image, &p);

  display_image(p);

  while(true){
    char c = (char)cv::waitKey(0); // Press  ESC on keyboard to  exit
    if( c == 27 ){
      break;}}




  return 0;}


