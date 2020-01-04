#include "AdvancedLaneFinderDefs.hpp"
#include "ImagePipeline.hpp"
#include "CLI/CLI.hpp"
#include "yaml_helpers.hpp"

// this is in it's own cpp file to reduce compilation time as yaml-cpp and CLI11
// apparently take a long time to build relative to the rest of the software.
int load_params(int argc, char* argv[], AdvancedLaneFinderParams* ap, ImagePipelineParams* ip){

  CLI::App app{"project 2 - advanced lane finding"};

  std::string config_yaml = "project2_config.yaml";
  app.add_option("-c, --config", config_yaml, "path to yaml configuration file");
  auto video_filename_opt = app.add_option("-f, --file", ip->video_filename, "video to process");
  app.add_flag("-l, --loop", ip->loop_video, "continuously loop video");
  app.add_flag("-t, --trackbar", ip->show_trackbars, "show parameter trackbars");
  app.add_flag("-r, --record", ip->record_video, "record processed video");

  CLI11_PARSE(app, argc, argv);

  YAML::Node config = YAML::LoadFile(config_yaml);

  if(config["video_filename"] && video_filename_opt->count() == 0){
    ip->video_filename = config["video_filename"].as<std::string>();}

  if(config["frame_rate_Hz"]){
    ip->frame_rate_Hz = config["frame_rate_Hz"].as<uint16_t>(); }

  if(config["trackbar_window"]){
    ip->trackbar_window = config["trackbar_window"].as<std::string>(); }

  if(config["image_window"]){
    ip->image_window = config["image_window"].as<std::string>(); }

  ap->show_trackbars = ip->show_trackbars;
  ap->trackbar_window = ip->trackbar_window;
  
  if(config["K"]){
    ap->K = config["K"].as<cv::Mat>(); }

  if(config["D"]){
    ap->D = config["D"].as<cv::Mat>();}

  if(config["M"]){
    ap->M = config["M"].as<cv::Mat>();}

  if(config["xm_per_pix"]){
    ap->xm_per_pix = config["xm_per_pix"].as<float>(); }

  if(config["ym_per_pix"]){
    ap->ym_per_pix = config["ym_per_pix"].as<float>(); }

  if(config["margin"]){
    ap->margin = config["margin"].as<uint16_t>();}

  if(config["minpix"]){
    ap->minpix = config["minpix"].as<uint16_t>(); }

  if(config["nwindows"]){
    ap->nwindows = config["nwindows"].as<uint16_t>(); }

  if(config["min_sobel"]){
    ap->min_sobel = config["min_sobel"].as<uint16_t>(); } //uint16_t because cli11 processes a uint8_t as an unsigned char
  
  if(config["max_sobel"]){
    ap->max_sobel = config["max_sobel"].as<uint16_t>(); }

  if(config["min_s"]){
    ap->min_s = config["min_s"].as<uint16_t>(); }

  if(config["max_s"]){
    ap->max_s = config["max_s"].as<uint16_t>(); }

  if(config["min_l"]){
    ap->min_l = config["min_l"].as<uint16_t>(); }

  if(config["max_l"]){
    ap->max_l = config["max_l"].as<uint16_t>(); }
  
  return 0;}