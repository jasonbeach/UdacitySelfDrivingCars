#include "AdvancedLaneFinder.hpp"

class AdvancedLaneFinder::AdvancedLaneFinderImpl{
  public:
    AdvancedLaneFinderImpl(const AdvancedLaneFinderParams& ap, const ImagePipelineParams& ip):
      ap_(ap), pipeline_(ip) {}

    void Run(){
      pipeline_.Run([this](const cv::Mat img){
        cv::Mat img_out;
        cv::cvtColor(img, img_out,CV_BGR2RGB );//simulated processing function
        return img_out; } ); }

  private:
    AdvancedLaneFinderParams ap_;
    ImagePipeline pipeline_;
};

AdvancedLaneFinder::AdvancedLaneFinder(const AdvancedLaneFinderParams& ap, const ImagePipelineParams& ip): 
  impl_{std::make_unique<AdvancedLaneFinderImpl>(ap, ip)} {}

AdvancedLaneFinder::AdvancedLaneFinder(AdvancedLaneFinder&&) = default;

AdvancedLaneFinder& AdvancedLaneFinder::operator=(AdvancedLaneFinder&&) = default;

AdvancedLaneFinder::~AdvancedLaneFinder() = default;

void AdvancedLaneFinder::Run(){ Impl()->Run();}