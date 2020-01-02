#pragma once
#include <memory>
#include "cv_common.hpp"

//Interface for Frame Source implementations
class FrameSourceImpl{
  public:
    FrameSourceImpl() = default;
    virtual ~FrameSourceImpl() = default;

    virtual bool IsOpen() const = 0;
    virtual cv::Mat GetFrame() = 0;
    virtual cv::Size GetFrameSize() const = 0;
    virtual int GetFrameRate() const = 0;
    virtual void Reset() = 0; };

//main FrameSource class
class FrameSource{
  public:
    FrameSource() = default;
    FrameSource(const std::string& dev_string);
    FrameSource(const FrameSource&) = delete;
    FrameSource(FrameSource&&);
    FrameSource& operator=(const FrameSource&) = delete;
    FrameSource& operator=(FrameSource&&);
    ~FrameSource();

    bool Open(const std::string& dev_string);
    bool IsOpen() const;
    cv::Mat GetFrame();
    cv::Size GetFrameSize() const;
    int GetFrameRate() const;
    void Reset();
    void Close();

  private:
    
    //helper methods for const correctness
    const FrameSourceImpl* Impl() const { return impl_.get(); }
    FrameSourceImpl* Impl() { return impl_.get(); }

    std::unique_ptr<FrameSourceImpl> impl_; };

//Interface for FrameSink implementations
class FrameSinkImpl{
  public:
    FrameSinkImpl() = default;
    virtual ~FrameSinkImpl() = default;

    virtual bool IsOpen() const = 0;
    virtual void WriteFrame(const cv::Mat& frame) = 0; };

// main FrameSink class
class FrameSink{
  public:
    FrameSink() = default;
    FrameSink(const std::string& dev_string, const cv::Size& frame_size, int frame_rate = 0);
    FrameSink(const FrameSink&) = delete;
    FrameSink(FrameSink&&);
    FrameSink& operator=(const FrameSink&) = delete;
    FrameSink& operator=(FrameSink&&);
    ~FrameSink();

    bool Open(const std::string& dev_string, const cv::Size& frame_size, int frame_rate = 0);
    bool IsOpen() const;
    void WriteFrame(const cv::Mat& frame);
    void Close();

  private:
    
    //helper methods for const correctness
    const FrameSinkImpl* Impl() const { return impl_.get(); }
    FrameSinkImpl* Impl() { return impl_.get(); }

    std::unique_ptr<FrameSinkImpl> impl_;   };


// Now to the actual pipeline class

struct ImagePipelineParams{
  std::string video_filename;
  std::string trackbar_window;
  std::string image_window;
  int frame_rate_Hz;
  bool loop_video;
  bool record_video;
  bool show_trackbars;
};

using ImageCallback = std::function<cv::Mat(const cv::Mat&)>;


class ImagePipeline{
  public:
    ImagePipeline();
    ImagePipeline(const ImagePipelineParams& p);
    ImagePipeline(const ImagePipeline&) = delete;
    ImagePipeline(ImagePipeline&&);
    ImagePipeline& operator=(const ImagePipeline&) = delete;
    ImagePipeline& operator=(ImagePipeline&&);
    ~ImagePipeline();

    void SetConfig(const ImagePipelineParams& p) ;
    void Run(ImageCallback frame_process_function);

  private:
    class ImagePipelineImpl;

    //helper methods for const correctness
    const ImagePipelineImpl* Impl() const { return impl_.get(); }
    ImagePipelineImpl* Impl() { return impl_.get(); }

    std::unique_ptr<ImagePipelineImpl> impl_; 
};