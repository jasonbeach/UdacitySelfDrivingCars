#include "config.hpp"
#include "yaml_helpers.hpp"

Params load_params(const std::string config_yaml){
  
  Params p;

  YAML::Node config = YAML::LoadFile(config_yaml);

  if(config["image_file_path"]){
    p.image_file_path = config["image_file_path"].as<std::string>(); 
    p.image_files = get_files(p.image_file_path);}


  if(config["K"]){
    p.K = config["K"].as<cv::Mat>(); }

  if(config["D"]){
    p.D = config["D"].as<cv::Mat>();}

  if(config["M"]){
    p.M = config["M"].as<cv::Mat>();
    p.M_inv = p.M.inv();}

  if(config["min_sobel"]){
    p.min_sobel = config["min_sobel"].as<uint16_t>(); }
  
  if(config["max_sobel"]){
    p.max_sobel = config["max_sobel"].as<uint16_t>(); }

  if(config["min_s"]){
    p.min_s = config["min_s"].as<uint16_t>(); }

  if(config["max_s"]){
    p.max_s = config["max_s"].as<uint16_t>(); }

  if(config["min_l"]){
    p.min_l = config["min_l"].as<uint16_t>(); }

  if(config["max_l"]){
    p.max_l = config["max_l"].as<uint16_t>(); }

  if(config["nwindows"]){
    p.nwindows = config["nwindows"].as<uint16_t>(); }

  if(config["minpix"]){
    p.minpix = config["minpix"].as<uint16_t>(); }

  if(config["xm_per_pix"]){
    p.xm_per_pix = config["xm_per_pix"].as<float>(); }

  if(config["ym_per_pix"]){
    p.ym_per_pix = config["ym_per_pix"].as<float>(); }

  return p;}