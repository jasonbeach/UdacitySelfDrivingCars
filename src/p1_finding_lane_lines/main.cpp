#include "cv_defs.hpp"
#include "finding_lane_lines.hpp"
#include "CLI/CLI.hpp"
#include <chrono>

void process_video(const std::string video_file, FindLaneParams* find_lane_params);

int main(int argc, char** argv ){

  FindLaneParams params;
  CLI::App app{"Project 1 Find lane lines"};

  std::string file_path = "./test_videos/project1/challenge.mp4";

  app.add_option("-f,--file", file_path, "path to files");
  app.add_flag("-l, --loop", params.loop_video, "continuously loop video");
  app.add_flag("-t, --trackbar", params.show_trackbars, "show parameter trackbars");
  app.add_flag("-c, --canny", params.show_canny, "show canny / hough transform output instead of final annotated imaged"); 

  CLI11_PARSE(app, argc, argv);

  process_video(file_path, &params);

  fmt::print("Final params: {}\n{}\n", file_path, params.str());
  return 0;}

void process_video(const std::string video_file, FindLaneParams* find_lane_params){
  if (find_lane_params ==  nullptr){
    fmt::print("no params provided!\n");
    return;}
  
  if(find_lane_params->show_trackbars){
    setup_windows(find_lane_params);}
  
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
  
  std::unique_ptr<cv::VideoWriter> video;
  if(!find_lane_params->loop_video){
    video = std::make_unique<cv::VideoWriter>("outcpp.mp4",CV_FOURCC('M','J','P','G'),frame_rate, cv::Size(frame_width,frame_height));}
 
  cv::Mat input_frame;
  cv::Mat output_frame;

  while(1){ 
    auto t1 = std::chrono::high_resolution_clock::now();
    cap >> input_frame; // Capture frame-by-frame
  
    if (input_frame.empty()){ // If the frame is empty, break immediately or reset to beginning
      if(find_lane_params->loop_video){
        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        continue;}
      else{
        break;}}

    FindLanes(input_frame, output_frame, *find_lane_params);
    
    if(video){
      video->write(output_frame);}
        
    cv::imshow( "Frame", output_frame ); // Display the resulting frame
  
    auto t2 = std::chrono::high_resolution_clock::now();

    auto process_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    
    auto delay_time_ms =  std::max((int64_t) 1, find_lane_params->frame_delay_ms - process_time_ms);

    char c = (char)cv::waitKey(delay_time_ms); // Press  ESC on keyboard to  exit
    if( c == 27 ){
      break;}}
 
  // free up the video capture and write object
  cap.release();
  if(video){
    video->release();}
 
  // Closes all the windows
  cv::destroyAllWindows();} // end process_video
