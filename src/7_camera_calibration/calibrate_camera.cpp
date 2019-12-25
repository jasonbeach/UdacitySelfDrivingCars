#include "fmt/core.h" 
#include "fmt/format.h"
#include <string>
#include "CLI/CLI.hpp"
#include "cv_common.hpp"
#include "calibrate.hpp"
#include "yaml_helpers.hpp"

struct CalibrateCamerasParams{
  std::string image_folder;
  std::string output_folder;
  std::vector<std::string> calibration_files;
  CameraPoints cp;
  cv::Mat K;
  cv::Mat D;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  int nx = 0;
  int ny = 0;
  bool undistort = false;
  bool show_images = false;};

int main(int argc, char** argv ){
  
  CalibrateCamerasParams p;
  CLI::App app{"checkerboard"};
  
  std::string config_yaml = "config.yaml";
  app.add_option("-f, --file", config_yaml, "path to yaml file");
  app.add_flag("-u, --undistort", p.undistort, "undistort images");
  app.add_flag("-s, --show", p.show_images, "show images");
  CLI11_PARSE(app, argc, argv);
  
  YAML::Node config = YAML::LoadFile(config_yaml);
  
  if(config["nx"]){
    p.nx = config["nx"].as<size_t>(); }

  if(config["ny"]){
    p.ny = config["ny"].as<size_t>();}

  if(config["image_folder"]){
    p.image_folder = config["image_folder"].as<std::string>();}
  
  if(config["output_folder"]){
    p.output_folder = config["output_folder"].as<std::string>();}

  if(config["calibration_files"]){
    p.calibration_files = config["calibration_files"].as<std::vector<std::string>>(); }

  if(p.calibration_files.empty()){
    fmt::print("No files given\n");
    return -1;}

  std::vector<std::string> full_filenames;
  for (auto& file : p.calibration_files){
    full_filenames.emplace_back(p.image_folder + "/" + file);}

  fmt::print("processing {} files\n", full_filenames.size());

  cv::Size chess_board_size{p.nx, p.ny};
  CameraPoints cp;
  bool first_image=true;
  cv::Size image_size;
  for(const auto & file : full_filenames){

    cv::Mat image = cv::imread( file, cv::IMREAD_COLOR );

    if ( !image.data ){
      fmt::print("processing {} failed\n", file);
      continue;}

    // assume all images are the same size as the first one.  Not a great assumption
    // but -\_/(:-)\_/-
    if(first_image){ 
      image_size.height = image.size[0];
      image_size.width = image.size[1];
      first_image = false;}
    
    fmt::print("processing: {} {}x{} {}\n", file, image_size.width, image_size.height, type2str(image.type()));

    extractPointsFromImage(image, chess_board_size, &cp, p.show_images);}
  
  cv::Mat K, D;
  std::vector<cv::Mat> rvecs, tvecs;

  fmt::print("image pts size: {} x 2, object pts size: {} x 3\n", 
    cp.image_points.size(), cp.object_points.size());
  
  cv::calibrateCamera(cp.object_points, cp.image_points, image_size, K, D, rvecs, tvecs);

  auto re = computeReprojectionErrors(cp.object_points, cp.image_points, rvecs, tvecs, K, D);

  fmt::print("\nreprojection error: {:.2f}\n", re);

  if(p.undistort){
    for(const auto& file : p.calibration_files){
      cv::Mat image = cv::imread( p.image_folder + "/" + file, cv::IMREAD_COLOR );
      cv::Mat udist_image;
      cv::undistort(image, udist_image, K, D);
      cv::imwrite(p.output_folder + "/" + file, udist_image ); } } 
  
  YAML::Node camera_cal_yaml;
  camera_cal_yaml["K_matrix"] = K;
  camera_cal_yaml["D_matrix"] = D;
  camera_cal_yaml["rvecs"] = rvecs;
  camera_cal_yaml["tvecs"] = tvecs;

  std::ofstream fout("camera_cal.yaml");
  fout << camera_cal_yaml;
  fout.flush();

  return 0;}


