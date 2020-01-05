#pragma once
#include "AdvancedLaneFinderDefs.hpp"
#include "ImagePipeline.hpp"

class AdvancedLaneFinder{
  public:   
    AdvancedLaneFinder(const AdvancedLaneFinderParams& ap, const ImagePipelineParams& ip);
    AdvancedLaneFinder(const AdvancedLaneFinder&) = delete;
    AdvancedLaneFinder(AdvancedLaneFinder&&);
    AdvancedLaneFinder& operator=(const AdvancedLaneFinder&) = delete;
    AdvancedLaneFinder& operator=(AdvancedLaneFinder&&);
    ~AdvancedLaneFinder();

    void Run();
  private:
    class AdvancedLaneFinderImpl;

    //helper methods for const correctness
    const AdvancedLaneFinderImpl* Impl() const { return impl_.get(); }
    AdvancedLaneFinderImpl* Impl() { return impl_.get(); }

    std::unique_ptr<AdvancedLaneFinderImpl> impl_; };