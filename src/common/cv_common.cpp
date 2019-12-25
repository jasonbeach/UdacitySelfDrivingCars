#include "cv_common.hpp"
#include <filesystem>
#include "fmt/core.h"

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;}

  r += "C";
  r += (chans+'0');

  return r;}

void ThresholdImage(cv::Mat input, cv::Mat& output, double min_val, double max_val){
  cv::threshold(input, output, max_val, 0, cv::THRESH_TOZERO_INV);
  cv::threshold(output, output, min_val, 255, cv::THRESH_BINARY);}

std::vector<std::string> get_files(const std::string& directory)
{
  std::vector<std::string> filenames;

  for (const auto & file : std::filesystem::directory_iterator(directory)){
    std::string ext = file.path().extension();
    if (ext == ".jpg" || ext == ".png" ){
      filenames.push_back(file.path().string());
      fmt::print("adding: {}\n", file.path().string());
    }
  }
//        std::cout << entry.path().string() << std::endl;
  return filenames;
}