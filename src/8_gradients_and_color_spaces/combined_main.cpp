#include <iostream>
#include <string>
#include <chrono>

#include "threshold_functions.hpp"
#include "fmt/core.h"
#include "CLI/CLI.hpp"
#include "yaml_helpers.hpp"

void display_image(const Params& p){

  auto t1 = std::chrono::high_resolution_clock::now();
  //cv::Mat abs_img = abs_sobel_thresh(p.image, p.orientation, p.min_thresh, p.max_thresh);
  cv::Mat mag_img = mag_threshold(p.image, p.mag_sobel_kernel, p.min_mag, p.max_mag);
  //cv::Mat dir_img = dir_threshold(mag_img, p.dir_sobel_kernel, p.min_angle, p.max_angle);
  //cv::Mat sobel_chan;

  //cv::bitwise_and(abs_img, mag_img, sobel_chan);
  //cv::bitwise_and(sobel_chan, dir_img, sobel_chan);

  cv::Mat s_chan = thresh_color(p.image, 2, p.min_s, p.max_s);
  cv::Mat l_chan = thresh_color(p.image, 1, p.min_l, p.max_l);
 
  cv::Mat color_chan;
  cv::bitwise_or(s_chan, l_chan, color_chan);

  cv::Mat empty = cv::Mat::zeros(s_chan.size(), CV_8UC1);
 
  cv::Mat arry[3] = {color_chan, mag_img /*sobel_chan*/, empty};

  cv::Mat merged;
  cv::merge(arry, 3, merged);
  auto t2 = std::chrono::high_resolution_clock::now();
  auto process_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
  fmt::print("processing time: {} ms\n", process_time_ms);
  cv::imshow("image", merged); } 
  
  // cv::Mat arry[3] = {abs_img, mag_img, dir_img};

  // cv::Mat merged;
  // cv::merge(arry, 3, merged);
  // cv::imshow("image", merged);}


int main(int argc, char** argv ){

  Params p; 
  CLI::App app{"color thresholds"};
  
  std::string config_yaml = "config.yaml";
  app.add_option("-f, --file", config_yaml, "path to yaml file");
  //app.add_flag("-u, --undistort", p.undistort, "undistort images");
  //app.add_flag("-s, --show", p.show_images, "show images");
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


