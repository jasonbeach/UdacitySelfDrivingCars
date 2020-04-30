/**
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 *
 * Created on: Dec 13, 2016
 * Author: Tiffany Huang
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <math.h>

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

constexpr double k2PI = 2.0 * M_PI;
constexpr double kD2R = M_PI / 180.0;
constexpr double kR2D = 180.0 / M_PI;



/**
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
  
  int id;     // Id of matching landmark in the map.
  double x;   // Local (vehicle coords) x position of landmark observation [m]
  double y;   // Local (vehicle coords) y position of landmark observation [m]
};


/**
 * ensures an angle is always between 0 and 2*pi 
 * @param (angle) angle in radian to wrap in between 0 and 2*pi
 * @output wrapped angle
 */
inline double wrap2pi(double angle){
  while (angle > k2PI){
    angle -= k2PI;
  }
  while (angle < 0){
    angle += k2PI;
  }
  return angle;
}








#endif  // HELPER_FUNCTIONS_H_
