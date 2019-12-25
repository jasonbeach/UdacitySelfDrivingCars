#include <iostream>
#include <string>

#include "threshold_functions.hpp"
#include "fmt/core.h"
#include "CLI/CLI.hpp"
#include "yaml_helpers.hpp"

void display_image(const Params& p){
  cv::Mat s_chan = thresh_color(p.image, 2, p.min_s, p.max_s);
  cv::Mat l_chan = thresh_color(p.image, 1, p.min_l, p.max_l);

  cv::Mat empty = cv::Mat::zeros(s_chan.size(), CV_8UC1);

  cv::Mat arry[3] = {s_chan, l_chan, empty};

  cv::Mat merged;
  cv::merge(arry, 3, merged);
  cv::imshow("image", merged);}

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

  cv::Mat image = cv::imread( p.image_files.at(0), cv::IMREAD_COLOR );

  if ( !image.data ){
    fmt::print("opening {} failed\n", p.image_files.at(0));
    exit(-1);}

  fmt::print("image size: {}x{} type: {}\n", image.cols, image.rows, type2str(image.type()));
  cv::undistort(image, p.image, p.K, p.D);


  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);

  cv::createTrackbar("min s","image", nullptr, 255, &adjust_s_min, &p);
  cv::setTrackbarPos("min s","image", p.min_s);
 
  cv::createTrackbar("max s","image", nullptr, 255, &adjust_s_max, &p);
  cv::setTrackbarPos("max s","image",p.max_s);

  cv::createTrackbar("min l","image", nullptr, 255, &adjust_l_min, &p);
  cv::setTrackbarPos("min l","image", p.min_l);
 
  cv::createTrackbar("max l","image", nullptr, 255, &adjust_l_max, &p);
  cv::setTrackbarPos("max l","image",p.max_l);

  cv::createTrackbar("image", "adjustments", nullptr, p.image_files.size() - 1, &change_image, &p);

  display_image(p);

  while(true){
    char c = (char)cv::waitKey(0); // Press  ESC on keyboard to  exit
    if( c == 27 ){
      break;}}




  return 0;}


