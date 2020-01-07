#include <iostream>
#include <string>
#include <chrono>
#include <random>

#include "threshold_functions.hpp"
#include "fmt/core.h"
#include "CLI/CLI.hpp"
#include "config.hpp"

int get_max_row(const cv::Mat& img){
  cv::Mat summed;
  cv::reduce(img, summed, 0, CV_REDUCE_SUM, CV_32S);

  cv::Point max_loc;

  cv::minMaxLoc(summed, nullptr, nullptr, nullptr, &max_loc);
  return max_loc.x;}


// from https://knowledge.udacity.com/questions/29265
  // we just solved 
  //         x = a*y^2 + b*y + c    eq(1) 
  // where a, b, c, x and y are in pixel space
  // we want 
  //         X = A*Y^2 + B*Y + C   eq(2) 
  // where A, B, C, X and Y are in real space meter)
  // let X = x*mx and Y = y*my
  // plugging these into eq 2: 
  //         x*mx = A*y^2*my^2 + B*y*my + C  eq(3)
  // or 
  //             A*y^2*my^2 + B*y*my + C   
  //         x = ----------   ------   -     eq(4)
  //                 mx         mx     mx
  //
  // by comparing similar terms between eq(1) and eq(4) we can infer that
  //             a * mx        b*mx       
  //         A = ------   B = -------  C = c*mx   eq(5)
  //              my^2          my
  //
  // radius of curvature in pixel space is:
  //
  //             (1+(2*a*y+b)^2)^1.5
  //        rc = -------------------        eq(6)
  //                    |2*a|
  //
  // or in real world coordinates
  //
  //             (1+(2*A*Y+B)^2)^1.5
  //        Rc = -------------------        eq(6)
  //                    |2*A|
  //

float compute_curvature_radius(float a, float b, float mx, float my, float y){
  float Y = y * my;
  float A = a * mx / SQ(my);
  float B = b * mx / my;
  float Rc = std::pow(SQ(2.0*A*Y + B)+1, 1.5) / std::abs(2.0*A);
  return Rc;}

cv::Mat find_lane_line(const cv::Mat& warped_image, const cv::Rect& roi, const Params& p, char lane, std::vector<cv::Point>* output_points, float* curvature_rad, float* offset){
  

  cv::Mat mask = cv::Mat::zeros(warped_image.size(), CV_8U); // all 0

  mask(roi) = 255;

  cv::Mat masked_warped;
  warped_image.copyTo(masked_warped, mask);

  int lane_line_center = get_max_row(masked_warped);

  int rows = warped_image.rows;

  int window_height = rows / p.nwindows;

  int current_x = lane_line_center - p.margin / 2; //upper left corner of rect
  int current_y = rows - window_height;            // upper left corner of rect
  
  cv::Mat empty = cv::Mat::zeros(warped_image.size(), CV_8U);
  cv::Mat nonzero_channel = cv::Mat::zeros(warped_image.size(), CV_8U);
  std::vector<cv::Point> nonzero_points;

  while(current_y >= 0 ){
    if (current_x < 0){
      current_x = 0; }
      
    if (current_x > warped_image.cols - p.margin){
      current_x = warped_image.cols - p.margin; }
    
    //fmt::print("x: {} y: {} margin: {} win height: {}\n", current_x, current_y, p.margin, window_height);
    cv::Rect window_roi = cv::Rect(current_x, current_y, p.margin, window_height);
    cv::Mat image_window = warped_image(window_roi);

    cv::Mat nonzero;
    cv::findNonZero(image_window, nonzero);
  
    for(int i = 0; i < nonzero.rows; ++i){
      cv::Point pt = nonzero.at<cv::Point>(i);
      pt.x += current_x;
      pt.y += current_y;
      nonzero_channel.at<uint8_t>(pt.y, pt.x) = 255;
      nonzero_points.push_back(pt);}

    if (nonzero.rows > p.minpix){
      cv::Mat x_pts;
      cv::extractChannel(nonzero, x_pts, 0);

      cv::Mat mean;
      x_pts.convertTo(mean, CV_32F);
      cv::reduce(mean, mean, 0, CV_REDUCE_AVG, CV_32F);

      current_x +=  mean.at<float>(0,0) - p.margin/2;}

    current_y -= window_height; 
    cv::rectangle(empty, window_roi, {255});} //draw  current row


  auto length = nonzero_points.size();
  cv::Mat A = cv::Mat::ones(length, 3, CV_32F);
  cv::Mat b(length, 1, CV_32F);

  //use 'y' as the independent variable
  for(size_t i = 0; i < nonzero_points.size(); ++i){
    A.at<float>(i, 0) = nonzero_points.at(i).y * nonzero_points.at(i).y;
    A.at<float>(i, 1) = nonzero_points.at(i).y;
    b.at<float>(i, 0) = nonzero_points.at(i).x;
  }

  fmt::print("A: {}, b: {}\n", A, b);

  cv::Mat x_sys;
  cv::solve(A, b, x_sys, cv::DECOMP_LU | cv::DECOMP_NORMAL);

  

  float a_pix = x_sys.at<float>(0,0);
  float b_pix = x_sys.at<float>(1,0);
  float c_pix = x_sys.at<float>(2,0);
  float y_pix = 720;

  float Rc = compute_curvature_radius(a_pix, b_pix, p.xm_per_pix, p.ym_per_pix, y_pix);

  *curvature_rad = Rc;
  *offset = a_pix*SQ(720.0) + b_pix*720.0 + c_pix;

  int loc = 30;
  if (lane == 'r'){
    loc = 60;}
  
  cv::putText(nonzero_channel,fmt::format("Radius:{:.2f}m", Rc),{10, loc},cv::FONT_HERSHEY_SIMPLEX,1, {255},3);

  std::vector<cv::Point> poly;
  for (int y = 0; y < nonzero_channel.rows; y+=20){
    float x = a_pix*SQ(y) + b_pix*y + c_pix;
    if(x >=0 && x <= nonzero_channel.cols){
      poly.emplace_back(x , y  );
      output_points->emplace_back(x,y);}
  }

  //std::vector<std::vector<cv::Point>> polys;
  //polys.push_back(poly);

  cv::polylines(nonzero_channel, poly, false, {255},3);

  cv::Mat arry[3] = {warped_image, nonzero_channel, empty};

  cv::Mat merged;
  cv::merge(arry, 3, merged);

  return merged;
}

void display_image(const Params& p){

  auto t1 = std::chrono::high_resolution_clock::now();
  cv::Mat abs_img = abs_sobel_thresh(p.image, p.min_sobel, p.max_sobel);
  
  cv::Mat s_chan = thresh_color(p.image, 2, p.min_s, p.max_s);
  cv::Mat l_chan = thresh_color(p.image, 1, p.min_l, p.max_l);
 
  cv::Mat color_chan;
  cv::Mat merged;
  
  cv::bitwise_or(s_chan, l_chan, color_chan);
  cv::bitwise_or(color_chan, abs_img, merged);

  if(p.demo){
    cv::Mat demo_merged;
    cv::Mat empty = cv::Mat::zeros(s_chan.size(), CV_8UC1);
    cv::Mat arry[3] = {color_chan, abs_img, empty};
    cv::merge(arry, 3, demo_merged);
    
    cv::imwrite(generate_output_filename(p.current_file, "_color_thresh"), demo_merged);
    cv::imwrite(generate_output_filename(p.current_file, "_binary_thresh"), merged);
  }  

  cv::Mat warped_img;
  cv::warpPerspective(merged, warped_img, p.M, merged.size());
  cv::threshold(warped_img, warped_img, 128, 255, CV_THRESH_BINARY);

  if(p.demo){ 
    cv::imwrite(generate_output_filename(p.current_file, "_warped"), warped_img);
  }

  //cv::Mat combined = nonzero_test(warped_img);
  std::vector<cv::Point> left_points, right_points;
  float Rc_left, Rc_right, l_off, r_off;
  cv::Mat left_line = find_lane_line(warped_img, cv::Rect(0,warped_img.rows/2,warped_img.cols/2, warped_img.rows/2), p, 'l', &left_points, &Rc_left, &l_off);
  cv::Mat right_line = find_lane_line(warped_img, cv::Rect(warped_img.cols/2, warped_img.rows/2, warped_img.cols/2, warped_img.rows/2), p, 'r', &right_points, &Rc_right, &r_off);

  float curvature = (Rc_right + Rc_left) / 2.0;

  float lane_center = (l_off + r_off) / 2.0;

  float center_offset = (lane_center - (warped_img.cols /2.0)) * p.xm_per_pix;



 
  auto t2 = std::chrono::high_resolution_clock::now();
  auto process_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
  fmt::print("processing time: {} ms\n", process_time_ms);

  cv::Mat combined = .5 * left_line + .5 * right_line;

  if(p.demo){ 
    cv::imwrite(generate_output_filename(p.current_file, "_warped_annotated"), combined);}

  cv::Mat warped_annotations = cv::Mat::zeros(warped_img.size(), CV_8UC3);
  std::vector<std::vector<cv::Point>> curve_points;
  curve_points.push_back(left_points);
  curve_points.at(0).insert(curve_points.at(0).end(), right_points.rbegin(), right_points.rend());
  
  cv::Mat full_annotated;
  if(curve_points.at(0).size() > 4){
    cv::fillPoly(warped_annotations, curve_points, cv::Scalar(0,255,0));
    
    cv::addWeighted(combined, 1.0, warped_annotations, .3, 0, combined);

    cv::Mat annotations;// = cv::Mat::zeros(undistorted.size(), CV_8U);
    cv::warpPerspective(warped_annotations, annotations, p.M_inv, warped_annotations.size());
      
    cv::putText(annotations, fmt::format("Radius: {:.2f}m", curvature),{10, 30},cv::FONT_HERSHEY_SIMPLEX,1, {0, 255, 0},3);
    cv::putText(annotations, fmt::format("Center offset: {:.2f}m", center_offset),{10, 60},cv::FONT_HERSHEY_SIMPLEX,1, {0, 255, 0},3);

    cv::addWeighted(p.image, 1.0, annotations,.3,0, full_annotated);  }

  if(p.demo){ 
    cv::imwrite(generate_output_filename(p.current_file, "_fully_annotated"), full_annotated);}

  cv::imshow("image", combined); 
  cv::imshow("unwarped image", full_annotated);

  } 
  

int main(int argc, char** argv ){

  CLI::App app{"color thresholds"};
  
  std::string config_yaml = "config.yaml";
  bool demo_mode = false;
  app.add_option("-f, --file", config_yaml, "path to yaml file");
  app.add_flag("-d, --demo", demo_mode, "demo mode");
  CLI11_PARSE(app, argc, argv);

  Params p = load_params(config_yaml);
  p.demo = demo_mode;

  if(p.image_files.empty()){
    fmt::print("No files given in: {}\n", p.image_file_path);
    return -1;}

  p.current_file = p.image_files.at(0);
  cv::Mat image = cv::imread( p.current_file, cv::IMREAD_COLOR );

  if ( !image.data ){
    fmt::print("opening {} failed\n", p.current_file );
    exit(-1);}

  fmt::print("image size: {}x{} type: {}\n", image.cols, image.rows, type2str(image.type()));
    
  cv::undistort(image, p.image, p.K, p.D);

  if(p.demo){
    cv::imwrite(generate_output_filename(p.current_file, "_undistort"), p.image);
  }

  
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("adjustments", CV_WINDOW_AUTOSIZE);

  cv::createTrackbar("sobel min","adjustments", nullptr, 255, &adjust_sobel_min, &p);
  cv::setTrackbarPos("sobel min","adjustments", p.min_sobel);
 
  cv::createTrackbar("sobel max","adjustments", nullptr, 255, &adjust_sobel_max, &p);
  cv::setTrackbarPos("sobel max","adjustments",p.max_sobel);

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


