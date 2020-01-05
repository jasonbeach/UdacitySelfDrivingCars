#include "ImagePipeline.hpp"
#include <filesystem>
#include <chrono>

using namespace std::chrono_literals;

class OpenCVFileFrameSourceImpl: public FrameSourceImpl {
  public:
    OpenCVFileFrameSourceImpl(const std::string& filename){
      video_reader_.open(filename); 

      if(!video_reader_.isOpened()){
        fmt::print("OpenCVFileFrameSourceImpl: Error opening file: {}\n", filename); 
        return;} 
 
      frame_size_.width = video_reader_.get(CV_CAP_PROP_FRAME_WIDTH); 
      frame_size_.height = video_reader_.get(CV_CAP_PROP_FRAME_HEIGHT); 
      frame_rate_ = video_reader_.get(CV_CAP_PROP_FPS);}

    bool IsOpen() const override{
      return video_reader_.isOpened();  }

    cv::Mat GetFrame() override{
      cv::Mat frame;
      video_reader_ >> frame;
      return frame; }

    cv::Size GetFrameSize() const override{
      return frame_size_; }

    int GetFrameRate() const override{
      return frame_rate_; }

    void Reset() override{
      video_reader_.set(cv::CAP_PROP_POS_FRAMES, 0); }

  private:
    cv::VideoCapture video_reader_; 
    cv::Size frame_size_ = {};
    int frame_rate_ = 0;};


FrameSource::FrameSource(const std::string& dev_string){
  Open(dev_string);}

FrameSource::FrameSource(FrameSource&&) = default;

FrameSource& FrameSource::operator=(FrameSource&&) = default;

FrameSource::~FrameSource() = default;

bool FrameSource::Open(const std::string& dev_string){ 
  // This should be augmented with a proper factory method,
  // but for now this is sufficient
  impl_ = std::make_unique<OpenCVFileFrameSourceImpl>(dev_string);
  if(IsOpen()){
    return true; }
  return false;}

bool FrameSource::IsOpen() const {return Impl()->IsOpen();}

cv::Mat FrameSource::GetFrame() {return Impl()->GetFrame();}

cv::Size FrameSource::GetFrameSize() const {return Impl()->GetFrameSize();}

int FrameSource::GetFrameRate() const {return Impl()->GetFrameRate();}

void FrameSource::Reset(){Impl()->Reset();}

void FrameSource::Close() {impl_.release(); }



class OpenCVFileFrameSinkImpl: public FrameSinkImpl {
  public:
    OpenCVFileFrameSinkImpl(const std::string& filename, const cv::Size& frame_size, int frame_rate){
      video_writer_.open(filename, CV_FOURCC('M','J','P','G'), frame_rate, frame_size); 

      if(!video_writer_.isOpened()){
        fmt::print("OpenCVFileFrameSinkImpl: Error opening file: {}\n", filename); 
        return; } }

    bool IsOpen() const override{
      return video_writer_.isOpened();  }

    void WriteFrame(const cv::Mat& frame) override{
      video_writer_.write(frame);}

  private:
    cv::VideoWriter video_writer_; };

FrameSink::FrameSink(const std::string& dev_string, const cv::Size& frame_size, int frame_rate){
  Open(dev_string, frame_size, frame_rate);}

FrameSink::FrameSink(FrameSink&&) = default;

FrameSink& FrameSink::operator=(FrameSink&&) = default;

FrameSink::~FrameSink() = default;

bool FrameSink::Open(const std::string& dev_string, const cv::Size& frame_size, int frame_rate){
  // This should be augmented with a proper factory method,
  // but for now this is sufficient
  impl_ = std::make_unique<OpenCVFileFrameSinkImpl>(dev_string, frame_size, frame_rate);
  if (IsOpen()){
    return true; }
  
  return false; }

bool FrameSink::IsOpen() const {
  if(!impl_){
    return false; }
  return Impl()->IsOpen();}

void FrameSink::WriteFrame(const cv::Mat& frame) {Impl()->WriteFrame(frame);}

void FrameSink::Close() {impl_.release(); }



class ImagePipeline::ImagePipelineImpl{
public:
  ImagePipelineImpl() = default;
  ImagePipelineImpl(const ImagePipelineParams& p): p_(p){ 
    if(!SetupPipeline()){
      fmt::print("ImagePipelineImpl: Error Setting up image pipeline\n");} }

  void SetConfig(const ImagePipelineParams& p){
    p_ = p; 

    if(!SetupPipeline()){
      fmt::print("SetConfig: Error Setting up image pipeline\n");}}

  cv::Size GetFrameSize() const{
    return video_reader_.GetFrameSize(); }

  void Run(ImageCallback frame_process_function){

    if(p_.show_trackbars){
      cv::createTrackbar("Frame Rate(Hz)", p_.trackbar_window, &p_.frame_rate_Hz, 50, nullptr);}

    while(1){ 
      auto t1 = std::chrono::high_resolution_clock::now();
      input_frame_ = video_reader_.GetFrame(); 
      
      if (input_frame_.empty()){ 
        if(p_.loop_video){    
          // we've reached the end and want to loop so reset video reader to beginning
          video_reader_.Reset(); 
          continue;}
        else{
          //we've reached the end of the video but don't want to loop so break
          break;}}  
      
      output_frame_ = frame_process_function(input_frame_);

      cv::imshow(p_.image_window, output_frame_); 
      
      if(video_writer_.IsOpen()){
        video_writer_.WriteFrame(output_frame_);}
          
      auto t2 = std::chrono::high_resolution_clock::now();
      // see how long it took to process the frame and sleep for the remainder
      // of the frame period      
      auto process_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
      int64_t frame_period_ms = static_cast<int64_t>(1.0/p_.frame_rate_Hz * 1000);
      auto delay_time_ms =  std::max((int64_t) 1, frame_period_ms - process_time_ms);
      fmt::print("process time: {}ms\n", process_time_ms);
      char c = (char)cv::waitKey(delay_time_ms); // Press  ESC on keyboard to  exit
      if( c == 27 ){
        break;}} }

private:

  bool SetupPipeline(){

    if(!video_reader_.Open(p_.video_filename)){
      return false; }  

    if(p_.record_video &&
      !video_writer_.Open(generate_output_filename(p_.video_filename), 
        video_reader_.GetFrameSize(), video_reader_.GetFrameRate())){
          return false;  }
        
    return true;}

  std::string generate_output_filename(const std::string& input_file){

    auto path = std::filesystem::path(input_file);
    auto stem_out = path.stem();
    stem_out += "_out";
    stem_out += path.extension();
    return stem_out.string();}

  ImagePipelineParams p_;
  FrameSource video_reader_; 
  FrameSink   video_writer_;
  cv::Mat input_frame_;
  cv::Mat output_frame_;};


ImagePipeline::ImagePipeline(): impl_{std::make_unique<ImagePipelineImpl>()} {}

ImagePipeline::ImagePipeline(const ImagePipelineParams& p): 
  impl_{std::make_unique<ImagePipelineImpl>(p)} {}

ImagePipeline::ImagePipeline(ImagePipeline&&) = default;

ImagePipeline& ImagePipeline::operator=(ImagePipeline&&) = default;

ImagePipeline::~ImagePipeline() = default;

void ImagePipeline::SetConfig(const ImagePipelineParams& p){
  Impl()->SetConfig(p);}

cv::Size ImagePipeline::GetFrameSize() const{
  return Impl()->GetFrameSize();}

void ImagePipeline::Run(ImageCallback frame_process_function){
  Impl()->Run(frame_process_function);}