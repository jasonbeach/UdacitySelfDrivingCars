#include "map.h"
#include <fstream>

void Map::AddLandMark(Landmark const& landmark){
  landmarks_.push_back(landmark);
}

Landmark Map::NearestLandmark(Eigen::Vector3d const& ref_pos, float max_dist) const{

  if(landmarks_.empty()){
    throw MapException("Empty Map Exception");
  }
  
  Landmark closest_landmark = *min_transformed_element(landmarks_.begin(), landmarks_.end(),
    [&](const Landmark& l) -> float {
      Eigen::Vector3d diff = l.location - ref_pos;
      return diff.x() * diff.x() + diff.y() * diff.y();
    } );

  double distance_to_landmark = (ref_pos - closest_landmark.location).norm();

  if(distance_to_landmark < max_dist){
    return closest_landmark;
  }
  return {};

  // This is *not* the most efficient way to find this 
  // as it computes 2N norm calculations when only N are needed
  // it also uses sqrt in the norm calc when for this application
  // it really isn't needed
  //return *std::min_element(landmarks_.begin(), landmarks_.end(),
  //[&](const Landmark& p1, const Landmark& p2){
  //  float p1_dist = (p1.location - ref_pos).norm();
  //  float p2_dist = (p2.location - ref_pos).norm();
  //  return p1_dist < p2_dist;
  //});
}

MapReader::MapReader(const std::string& file_name){

  // Get file of map
  std::ifstream in_file_map(file_name, std::ifstream::in);
  // Return if we can't open the file
  if (!in_file_map) {
    throw MapException("File Not Found");
  }
  
  // Declare single line of map file
  std::string line_map;

  // Run over each single line
  while (std::getline(in_file_map, line_map)) {

    std::istringstream iss_map(line_map);

    // Declare landmark values and ID
    double landmark_x_f = 0;
    double landmark_y_f = 0;
    int id_i = 0;

    // Read data from current line to values
    iss_map >> landmark_x_f;
    iss_map >> landmark_y_f;
    iss_map >> id_i;

    // Declare single_landmark
    Landmark landmark_temp;

    // Set values
    landmark_temp.id = id_i;
    landmark_temp.location.x()  = landmark_x_f;
    landmark_temp.location.y()  = landmark_y_f;

    // Add to landmark list of map
    map_.AddLandMark(landmark_temp);
  }
}

Map MapReader::GetMap() const{
  return map_;
}