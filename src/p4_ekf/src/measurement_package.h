#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

struct MeasurementPackage {

  enum class SensorType{
    LASER,
    RADAR
  } sensor_type;

  uint64_t timestamp;

  Eigen::Vector3d raw_measurements;
};

#endif // MEASUREMENT_PACKAGE_H_
