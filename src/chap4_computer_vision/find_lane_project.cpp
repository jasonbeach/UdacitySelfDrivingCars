#include "fmt/core.h" 
#include <string>
#include <opencv2/opencv.hpp>
#include "cv_helpers.hpp"
#include <filesystem>
#include "CLI/CLI.hpp"

namespace fs = std::filesystem;
 
static double constexpr kPi = 3.14159265359;
static double constexpr kD2R = kPi / 180.0;

struct FindLaneParams
{

};

void FindLanes(const cv::Mat& input, cv::Mat& output, const FindLaneParams& parms)
{
  auto y_size = input.size[0];
  auto x_size = input.size[1];

  static cv::Mat gray_image; 
  cv::cvtColor(input, gray_image, cv::COLOR_BGR2GRAY);
  
  // Define a kernel size for Gaussian smoothing / blurring
  // Note: this step is optional as cv::Canny() applies a 5x5 Gaussian internally
  cv::Size kernel_size {5, 5};
  static cv::Mat blur_gray; 
  cv::GaussianBlur(gray_image, blur_gray, kernel_size, 0);

  // Define parameters for Canny and run it
  int low_threshold = 50;
  int high_threshold = 140;
  static cv::Mat canny; 
  cv::Canny(blur_gray, canny, low_threshold, high_threshold);

  const int top_width = 20;
  std::vector<std::vector<cv::Point>> vertices {{{0,y_size},{x_size/2-top_width,(int)(y_size*.6)},{x_size/2+top_width,(int)(y_size*.6)},{x_size,y_size}}};

  static cv::Mat mask;
  mask = cv::Mat::zeros(canny.size(),CV_8UC1);

  cv::fillPoly(mask, vertices, cv::Scalar(255));

  static cv::Mat masked_canny;
  canny.copyTo(masked_canny, mask);

  // Define the Hough transform parameters
  // Make a blank the same size as our image to draw on
  int rho = 2;
  double theta = kPi/180;
  int threshold = 15;
  int min_line_length = 40;
  int max_line_gap = 20;

  static cv::Mat masked_canny_color;
  cv::cvtColor(masked_canny, masked_canny_color, CV_GRAY2BGR);

  //create blank bgr image to draw lines on
  cv::Mat color_lines = cv::Mat::zeros(masked_canny_color.size(), masked_canny_color.type()); 
  
  // Run Hough on edge detected image
  std::vector<cv::Vec4i> hough_lines;
  cv::HoughLinesP(masked_canny, hough_lines, rho, theta, threshold, min_line_length, max_line_gap);
  
  for (const auto& line : hough_lines){
    double dot_product = line[0]*line[2] + line[1] * line[3];
    cv::Point2i p1 {line[0], line[1]};
    cv::Point2i p2 {line[2], line[3]};
    cv::Vec2i line_vector{ std::abs(p2.y - p1.y), std::abs(p2.x - p1.x)}; // convert points into a vector
    double length = cv::norm(line_vector);


    double theta = std::acos(line_vector[0] / length);
    if( theta < (60.0*kD2R) && theta > (30 * kD2R)){
      cv::line(color_lines, p1, p2, cv::Scalar(0,0,255), 3, CV_AA);}}

  // # Draw the lines on the edge image
  cv::addWeighted(color_lines, 0.8, masked_canny_color, 1, 0, output);} //end FindLanes

void process_files(const std::string& image_path){
  for(auto& file: fs::directory_iterator(image_path)){
    const auto& filename = file.path();
    fmt::print("processing file: {}\n", filename.filename().string());
    if(filename.extension() != ".jpg"){
      continue;}  

    cv::Mat image = cv::imread( filename.string(), cv::IMREAD_COLOR );
  
    if ( !image.data ){
      fmt::print("No image data \n");
      return;}

    auto y_size = image.size[0];
    auto x_size = image.size[1];
    fmt::print("image size: {}x{} type: {}\n", x_size, y_size, type2str(image.type()));

    cv::Mat combo;

    auto t1 = std::chrono::high_resolution_clock::now();
    FindLanes(image, combo, FindLaneParams());
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    fmt::print("Processing time: {} us\n", duration);
    cv::imshow(filename.filename().string(), combo);}
    
  cv::waitKey(0);}

void process_video(const std::string video_file){
  // Create a VideoCapture object and use camera to capture the video
  cv::VideoCapture cap(video_file); 
 
  // Check if camera opened successfully
  if(!cap.isOpened()){
    fmt::print("Error opening video stream\n"); 
    return;} 
 
  // Default resolution of the frame is obtained.The default resolution is system dependent. 
  int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
  int frame_rate = cap.get(CV_CAP_PROP_FPS);

  fmt::print("frame size: {}x{} frame rate: {} \n",frame_width, frame_height, frame_rate);
  // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file. 
  cv::VideoWriter video("outcpp.mp4",CV_FOURCC('M','J','P','G'),frame_rate, cv::Size(frame_width,frame_height)); 
  cv::Mat input_frame;
  cv::Mat output_frame;
  FindLaneParams find_lane_params;

  while(1){ 
    cap >> input_frame; // Capture frame-by-frame
  
    if (input_frame.empty()){ // If the frame is empty, break immediately
      break;}

    FindLanes(input_frame, output_frame, find_lane_params);

    video.write(output_frame);
        
    cv::imshow( "Frame", output_frame ); // Display the resulting frame
  
    char c = (char)cv::waitKey(1); // Press  ESC on keyboard to  exit
    if( c == 27 ){
      break;}}
 
  // free up the video capture and write object
  cap.release();
  video.release();
 
  // Closes all the windows
  cv::destroyAllWindows();}

int main(int argc, char** argv ){
  
  CLI::App app{"Project 1 Find lane lines"};

  bool do_images = false;
  bool do_videos = false;
  std::string image_path = "/home/jason/devel/udacity/cpp/test_images";
  std::string video_path = "/home/jason/devel/udacity/cpp/test_videos/solidWhiteRight.mp4";
  std::string file_path;
  auto image_flag = app.add_flag("-i,--image", do_images, "process image frames");
  auto video_flag = app.add_flag("-v, --video", do_videos, "process video");
  image_flag->excludes(video_flag);
  video_flag->excludes(image_flag);
  app.add_option("-f,--file", file_path, "path to files");
  

  CLI11_PARSE(app, argc, argv);

  if(do_images){
    if(app.count("--file") == 0){
      file_path = image_path;}

    process_files(file_path);}

  if(do_videos){
    if(app.count("--file") == 0){
      file_path = video_path;}
    process_video(file_path);}
  

  return 0;
}

