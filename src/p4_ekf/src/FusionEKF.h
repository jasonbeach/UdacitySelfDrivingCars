#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

   /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  Eigen::Vector4d CurrentState() const;

 private:
  void Initialize(const MeasurementPackage& mp);
  void Predict(double timestamp);
  void LidarUpdate(const Eigen::Vector2d& z);
  void RadarUpdate(const Eigen::Vector3d& z);
  Eigen::Matrix<double, 3, 4> UpdateRadarJacobian();
  
  // check whether the tracking toolbox was iProcessMeasurementnitialized or not (first measurement)
  bool is_initialized_ = false;

  // previous timestamp
  uint64_t previous_timestamp_ = 0;

  Eigen::Vector4d x_; //Px, Py, Vx, Vy
  Eigen::Matrix4d P_;
  Eigen::Matrix4d F_;
  Eigen::Matrix<double, 4, 2> G_;

  Eigen::Matrix2d R_laser_;
  Eigen::Matrix3d R_radar_;
  Eigen::Matrix<double, 2, 4> H_laser_;
  Eigen::Matrix2d Qv_;
  size_t meas_num_ = 0;

};

#endif // FusionEKF_H_
