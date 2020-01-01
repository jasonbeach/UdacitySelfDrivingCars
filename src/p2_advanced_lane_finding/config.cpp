#include "config.hpp"
#include "CLI/CLI.hpp"
#include "yaml_helpers.hpp"

Params load_params(int argc, char* argv[]){

  CLI::App app{"project 2 - advanced lane finding"};

  Params p;

  std::string config_yaml = "project2_config.yaml";
  app.add_option("-c, --config", config_yaml, "path to yaml configuration file");
  auto video_filename_opt = app.add_option("-f, --file", p.video_filename, "video to process");
  app.add_flag("-l, --loop", p.loop_video, "continuously loop video");
  app.add_flag("-t, --trackbar", p.show_trackbars, "show parameter trackbars");
  app.add_flag("-r, --record", p.record_video, "record processed video");

  try {
    app.parse(argc, argv);}
  catch(const CLI::ParseError &e) {
    fmt::print("CLI Parse error: {}\n", e.what());
    throw e;}

  YAML::Node config = YAML::LoadFile(config_yaml);

  if(config["video_filename"] && video_filename_opt->count() == 0){
    p.video_filename = config["video_filename"].as<std::string>();}

  if(config["K"]){
    p.K = config["K"].as<cv::Mat>(); }

  if(config["D"]){
    p.D = config["D"].as<cv::Mat>();}

  if(config["M"]){
    p.M = config["M"].as<cv::Mat>();}

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

  if(config["margin"]){
    p.margin = config["margin"].as<uint16_t>();}

  if(config["minpix"]){
    p.minpix = config["minpix"].as<uint16_t>(); }

  if(config["xm_per_pix"]){
    p.xm_per_pix = config["xm_per_pix"].as<float>(); }

  if(config["ym_per_pix"]){
    p.ym_per_pix = config["ym_per_pix"].as<float>(); }

  p.initialized = true;
  return p;}