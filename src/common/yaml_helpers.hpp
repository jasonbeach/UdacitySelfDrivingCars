#pragma once
#include "yaml-cpp/yaml.h"
#include <opencv2/opencv.hpp>

namespace YAML {
template<>
struct convert<cv::Mat> {
  static Node encode(const cv::Mat& mat) {
    Node node;
    node["rows"] = mat.rows;
    node["cols"] = mat.cols;
    node["type"] = mat.type();
    for(int row = 0; row < mat.rows; ++row){
      for(int col = 0; col < mat.cols; ++col){
        node["data"].push_back(mat.at<double>(row,col));
      }
    }
    return node;
  }

  static bool decode(const Node& node, cv::Mat& mat) {
    if(!node["data"] && !node["rows"] && !node["cols"] && !node["type"] && !node["data"].IsSequence()) {
      return false;
    }

    size_t rows = node["rows"].as<size_t>();
    size_t cols = node["cols"].as<size_t>();
    int type = node["type"].as<int>();

    if (node["data"].size() != rows * cols){
      return false;
    }
    
    mat.create(rows, cols, type);
    for(int row = 0; row < mat.rows; ++row){
      for(int col = 0; col < mat.cols; ++col){
        mat.at<double>(row, col) = node["data"][row*mat.cols + col].as<double>();
      }
    }
    return true;
  }
};
}