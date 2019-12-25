#include "fmt/core.h" 
#include "fmt/format.h"
#include <string>
#include "CLI/CLI.hpp"
#include "yaml-cpp/yaml.h"
#include "cv_common.hpp"
#include "calibrate.hpp"
#include "yaml_helpers.hpp"

struct ImageParams{
  cv::Mat image;
  std::vector<cv::Point2i> src_points;
  std::vector<cv::Point2i> dst_points;
  cv::Mat M;
};

struct PerspectiveParams{
  std::string image_file;
  cv::Mat K;
  cv::Mat D;
  std::string output_yaml_file;};

void handle_mouse_click(int event, int x, int y, int flags, void* userdata)
{
  (void) flags;
  auto ip = static_cast<ImageParams*> (userdata);
  cv::Mat annotated_image;
  ip->image.copyTo(annotated_image);
  cv::putText(annotated_image,fmt::format("({}x{})",x,y),{100,100},cv::FONT_HERSHEY_SIMPLEX,1, {0,255,0},3);
  if  ( event == cv::EVENT_LBUTTONDOWN )
  {
      fmt::print("Left button of the mouse is clicked - position ({}, {})\n",x ,y);
      if(ip->src_points.size() < 4){
        ip->src_points.emplace_back(x,y);}
      if (ip->src_points.size() == 4){
        auto pt1 = ip->src_points[0];
        auto pt2 = ip->src_points[1];
        //auto pt3 = ip->src_points[2];
        auto pt4 = ip->src_points[3];
        auto pt = pt2 - pt1;

        ip->dst_points.push_back(pt1);
        int len = sqrt(pt.x * pt.x + pt.y * pt.y );
        fmt::print("len: {}\n", len);      
        pt.x = pt1.x;
        pt.y = pt1.y - len;
        ip->dst_points.push_back(pt);
        pt.x = pt4.x;
        ip->dst_points.push_back(pt);
        ip->dst_points.push_back(pt4);
        
        }
  }
  else if  ( event == cv::EVENT_RBUTTONDOWN )
  {
      fmt::print("Right button of the mouse is clicked - position ({},{})\n", x, y );
  }
  
  auto src_points = ip->src_points;
  auto dst_points = ip->dst_points;
  if(!src_points.empty() && src_points.size() < 4){
    src_points.emplace_back(x,y);}
  else if(!dst_points.empty() && dst_points.size() < 4){
    dst_points.emplace_back(x,y);}

  if(dst_points.size() == 4){
    std::vector<cv::Point2f> src_points_f;
    std::vector<cv::Point2f> dst_points_f;
    for(int i = 0; i < 4; ++i){
      src_points_f.push_back(src_points[i]);
      dst_points_f.push_back(dst_points[i]);
    }

    ip->M = cv::getPerspectiveTransform(src_points_f, dst_points_f);
    cv::warpPerspective(annotated_image, annotated_image, ip->M, annotated_image.size());
  }

  for(const auto& pt : src_points){
    cv::drawMarker(annotated_image, pt, {255,0,0},cv::MARKER_STAR,10,1);}
  cv::polylines(annotated_image, src_points, (ip->src_points.size() == 4), {255,0,0},1 );
  
  for(const auto& pt : dst_points){
    cv::drawMarker(annotated_image, pt, {0,0,255},cv::MARKER_STAR,10,1);}
  cv::polylines(annotated_image, dst_points, (ip->dst_points.size() == 4), {0,0,255},1 );



  cv::imshow("ImageWindow", annotated_image);
}

int main(int argc, char** argv ){
  
  PerspectiveParams p;
  CLI::App app{"perspective transforms"};
  
  std::string config_yaml = "config.yaml";
  app.add_option("-f, --file", config_yaml, "path to yaml file");
  CLI11_PARSE(app, argc, argv);
  
  YAML::Node config = YAML::LoadFile(config_yaml);
  
  if(config["K_matrix"]){
    p.K = config["K_matrix"].as<cv::Mat>(); }

  if(config["D_matrix"]){
    p.D = config["D_matrix"].as<cv::Mat>();}

  if(config["image_file"]){
     p.image_file = config["image_file"].as<std::string>();}
  
  if(config["output_yaml_file"]){
     p.output_yaml_file = config["output_yaml_file"].as<std::string>();}
  
  ImageParams ip;
  auto image = cv::imread( p.image_file, cv::IMREAD_COLOR );
  cv::undistort(image,ip.image, p.K, p.D);

  if ( !ip.image.data ){
    fmt::print("processing {} failed\n", p.image_file);
      return -1;}

  cv::Size image_size;
  image_size.height = ip.image.size[0];
  image_size.width = ip.image.size[1];
  
  fmt::print("processing: {}, {}x{} {}\n", p.image_file, image_size.width, image_size.height, type2str(ip.image.type()));

  cv::namedWindow("ImageWindow");
  cv::setMouseCallback("ImageWindow", handle_mouse_click, &ip);

     //show the image
  cv::imshow("ImageWindow", ip.image);

     // Wait until user press some key
  cv::waitKey(0);

  YAML::Node perspective_transform_yaml;
  perspective_transform_yaml["M_matrix"] = ip.M;

  std::ofstream fout(p.output_yaml_file);
  fout << perspective_transform_yaml;
  fout.flush();

  return 0;}