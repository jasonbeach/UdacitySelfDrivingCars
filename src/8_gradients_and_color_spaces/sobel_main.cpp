#include <string>
#include "threshold_functions.hpp"
#include "fmt/core.h" 
#include "CLI/CLI.hpp"
#include "yaml_helpers.hpp"

void display_image(const Params& p){
  cv::Mat img = abs_sobel_thresh(p.image, p.orientation, p.min_thresh, p.max_thresh);
  cv::imshow("image", img);}

int main(int argc, char** argv ){

  Params p; 

  CLI::App app{"sobel"};
  std::string config_yaml = "config.yaml";
  app.add_option("-f, --file", config_yaml, "path to yaml file");
  app.add_option("-o, --orient", p.orientation, "sobel direction");
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

  cv::Mat image = cv::imread( p.image_files[0], cv::IMREAD_COLOR );

  if ( !image.data ){
    fmt::print("opening {} failed\n", p.image_files[0]);
    exit(-1);}

  fmt::print("image size: {}x{} type: {}\n", image.cols, image.rows, type2str(image.type()));
  cv::undistort(image, p.image, p.K, p.D);

  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);

  cv::createTrackbar("min thresh","image", nullptr, 255, &adjust_sobel_min, &p);
  cv::setTrackbarPos("min thresh","image", p.min_thresh);
 
  cv::createTrackbar("max thresh","image", nullptr, 255, &adjust_sobel_max, &p);
  cv::setTrackbarPos("max thresh","image",p.max_thresh);

  cv::createTrackbar("image", "adjustments", nullptr, p.image_files.size() - 1, &change_image, &p);

  display_image(p);

  while(true){
    char c = (char)cv::waitKey(0); // Press  ESC on keyboard to  exit
    if( c == 27 ){
      break;}}

  return 0;}


