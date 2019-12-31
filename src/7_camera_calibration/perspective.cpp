#include "fmt/core.h" 
#include "fmt/format.h"
#include <string>
#include "CLI/CLI.hpp"
#include "yaml-cpp/yaml.h"
#include "cv_common.hpp"
#include "calibrate.hpp"
#include "yaml_helpers.hpp"

struct ConfigParams{
  cv::Mat image;
  std::vector<std::string> image_files;
  std::vector<cv::Point2i> src_points;
  std::vector<cv::Point2i> dst_points;
  std::vector<cv::Point> x_line;
  std::vector<cv::Point> y_line;
  cv::Mat M;
  cv::Mat K;
  cv::Mat D;
  std::string perspective_output_file;
};

// notwithstanding this function being a fantastic example of horribly,
// abysmally written code, it manages to get the job done. Do not look here 
// for the right way to do things.
void handle_mouse_click(int event, int x, int y, int flags, void* userdata){
  (void) flags;
  auto p = static_cast<ConfigParams*> (userdata);
  cv::Mat annotated_image;
  p->image.copyTo(annotated_image);
  cv::putText(annotated_image,fmt::format("({}x{})",x,y),{100,100},cv::FONT_HERSHEY_SIMPLEX,1, {0,255,0},3);
  
  if (event == cv::EVENT_LBUTTONDOWN ){
    fmt::print("Left button of the mouse is clicked - position ({}, {})\n",x ,y);
    static bool has_trans = false;
    if(p->src_points.size() < 4){
      p->src_points.emplace_back(x,y);
      has_trans = false;}

    // yes this is horrible. 
    if(has_trans){
      if(p->y_line.size() < 2)
        p->y_line.emplace_back(x,y); }

    if (p->src_points.size() == 4 && !has_trans){
      fmt::print("src points size: {}\n", p->src_points.size());
      auto pt1 = p->src_points[0];
      auto pt2 = p->src_points[1];
      //auto pt3 = p->src_points[2];
      auto pt4 = p->src_points[3];
      auto pt = pt2 - pt1;

      p->dst_points.push_back(pt1);
      int len = sqrt(pt.x * pt.x + pt.y * pt.y );
      fmt::print("len: {}\n", len);      
      pt.x = pt1.x;
      pt.y = pt1.y - len;
      p->dst_points.push_back(pt);
      pt.x = pt4.x;
      p->dst_points.push_back(pt);
      p->dst_points.push_back(pt4);
      fmt::print("x dist: {} pixels\n", std::abs(pt4.x - pt1.x));
      has_trans = true; }  }
  else if  ( event == cv::EVENT_RBUTTONDOWN ){
    fmt::print("Right button of the mouse is clicked - position ({},{})\n", x, y );}
  
  auto src_points = p->src_points;
  auto dst_points = p->dst_points;
  if(!src_points.empty() && src_points.size() < 4){
    src_points.emplace_back(x,y);}
  else if(!dst_points.empty() && dst_points.size() < 4){
    dst_points.emplace_back(x,y);}

  if(dst_points.size() == 4){
    std::vector<cv::Point2f> src_points_f;
    std::vector<cv::Point2f> dst_points_f;
    for(int i = 0; i < 4; ++i){
      src_points_f.push_back(src_points[i]);
      dst_points_f.push_back(dst_points[i]); }

    p->M = cv::getPerspectiveTransform(src_points_f, dst_points_f);
    cv::warpPerspective(annotated_image, annotated_image, p->M, annotated_image.size());
    if(event == cv::EVENT_LBUTTONDOWN ){
      fmt::print("Transform is: {}\n", p->M);}}

  for(const auto& pt : src_points){
    cv::drawMarker(annotated_image, pt, {255,0,0},cv::MARKER_STAR,10,1);}
  cv::polylines(annotated_image, src_points, (p->src_points.size() == 4), {255,0,0},1 );
  
  for(const auto& pt : dst_points){
    cv::drawMarker(annotated_image, pt, {0,0,255},cv::MARKER_STAR,10,1);}
  cv::polylines(annotated_image, dst_points, (p->dst_points.size() == 4), {0,0,255},1 );


  if(p->y_line.size() == 1){
    std::vector<cv::Point> plot_line;
    plot_line.push_back(p->y_line.at(0));
    plot_line.emplace_back(x,y);
    cv::polylines(annotated_image, plot_line, false, {255,0,0},1);
  }

  if(p->y_line.size() == 2){
    cv::polylines(annotated_image, p->y_line, false, {255,0,0 },3);
    if  ( event == cv::EVENT_LBUTTONDOWN ){
      fmt::print("y_line len: {} pixels\n", std::abs(p->y_line.at(0).y - p->y_line.at(1).y));}}

  cv::imshow("ImageWindow", annotated_image);}



void display_image( const ConfigParams& p){
  
  cv::imshow("ImageWindow", p.image);}

void change_image( int count, void* param){
  ConfigParams* p = static_cast<ConfigParams*>(param);

  cv::Mat image = cv::imread( p->image_files.at(count), cv::IMREAD_COLOR );
  if ( !image.data ){
    fmt::print("processing {} failed\n", p->image_files.at(0));
    image = cv::Mat::zeros(500,500,CV_8U);}
  else {
    fmt::print("processing: {}, {}x{} {}\n", p->image_files.at(count), image.cols, image.rows, type2str(p->image.type()));
    cv::undistort(image, p->image, p->K, p->D);}
    
  p->src_points.clear();
  p->dst_points.clear();
  p->x_line.clear();
  p->y_line.clear();
  display_image(*p);}

int main(int argc, char** argv ){
  
  ConfigParams p;
  CLI::App app{"perspective transforms"};
  
  std::string config_yaml = "config.yaml";
  app.add_option("-f, --file", config_yaml, "path to yaml file");
  CLI11_PARSE(app, argc, argv);
  
  YAML::Node config = YAML::LoadFile(config_yaml);
  
  if(config["K"]){
    p.K = config["K"].as<cv::Mat>(); }

  if(config["D"]){
    p.D = config["D"].as<cv::Mat>();}

  std::string image_file_path;
  if(config["image_file_path"]){
    image_file_path = config["image_file_path"].as<std::string>(); 
    p.image_files = get_files(image_file_path);}
  
  if(config["perspective_output_file"]){
     p.perspective_output_file = config["perspective_output_file"].as<std::string>();}
  
  if(p.image_files.empty()){
    fmt::print("no images found in: {}\n", image_file_path);
    return -1; }

  cv::namedWindow("ImageWindow");
  
  cv::setMouseCallback("ImageWindow", handle_mouse_click, &p);
  cv::createTrackbar("image", "ImageWindow", nullptr, p.image_files.size() -1, change_image, &p);

  change_image(0, &p);
 
  while(true){
    char c = (char)cv::waitKey(0); // Press  ESC on keyboard to  exit
    if( c == 27 ){
      break;}}

  YAML::Node perspective_output_yaml;
  perspective_output_yaml["M"] = p.M;

  std::ofstream fout(p.perspective_output_file);
  fout << perspective_output_yaml;
  fout.flush();

  return 0;}