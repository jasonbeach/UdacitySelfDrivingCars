#include <string>

#include "threshold_functions.hpp"
#include "fmt/core.h"
#include "CLI/CLI.hpp"
#include "yaml_helpers.hpp"

void display_image(const Params& p){
  cv::Mat grad_img = dir_threshold(p.image, p.dir_sobel_kernel, p.min_angle, p.max_angle);

  cv::imshow("image", grad_img);}


int main(int argc, char** argv ){

  Params p; 
  CLI::App app{"sobel"};
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

  cv::createTrackbar("min thresh","image", nullptr, 157, &adjust_min_angle, &p);
  cv::setTrackbarPos("min thresh","image", p.min_angle * 100.0);
 
  cv::createTrackbar("max thresh","image", nullptr, 157, &adjust_max_angle, &p);
  cv::setTrackbarPos("max thresh","image",p.max_angle * 100);

  cv::createTrackbar("image", "adjustments", nullptr, p.image_files.size() - 1, &change_image, &p);

  display_image(p);

  while(true){
    char c = (char)cv::waitKey(0); // Press  ESC on keyboard to  exit
    if( c == 27 ){
      break;}}

  return 0;}


