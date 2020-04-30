/**
 * map.h
 *
 * Created on: Apr 18, 2020
 * Author: Jason Beach
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <Eigen/Dense>

// this really should go somewhere else since it's designed to be very generic
// it'm meant to minimic std::minimum_element, but allows you to transform the 
// element in some way. We could use std::minimum_element, but the transform function
// would be evaluated twice as many times as it needed to be as it's results aren't
// cached. This caches the value. 
template<class ForwardIt, class UnaryOperation>
ForwardIt min_transformed_element(ForwardIt first, ForwardIt last, UnaryOperation unary_op)
{
  if (first == last) return last;

  ForwardIt smallest = first;
  auto smallest_val = unary_op(*first);

  ++first;
  for (; first != last; ++first) {
    auto first_val = unary_op(*first);
    if (first_val < smallest_val) {
      smallest = first;
      smallest_val = first_val;
    }
  }
  return smallest;
}

struct Landmark {
  int id = 0; // Landmark ID
  Eigen::Vector3d location; //Landmark position in homogeneous (global) coordinates
};

class Map{
public:
  void AddLandMark(Landmark const& landmark);
  Landmark NearestLandmark(Eigen::Vector3d const& ref_pos, float max_dist = std::numeric_limits<float>::max()) const ;

private:
  std::vector<Landmark> landmarks_;
};


struct MapException : public std::exception{
  MapException() = default;
  MapException(const std::string& message): message_(message) {}
  const char* what() const throw(){return message_.c_str();}
private:
  std::string message_;
};

/**
 * Reads map data from a file.
 * @param filename Name of file containing map data.
 * Throws exception if opening and reading file was successful
 */

class MapReader{
public:
  MapReader(const std::string& file_name);

  Map GetMap() const;
private:
  Map map_;
};



#endif  // MAP_H_
