#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <iomanip>

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {

  //State covariance matrix
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 100, 0,
        0, 0, 0, 100;
  
  // State Transition matrix.  
  // [ 1  0 dt  0 ]
  // [ 0  1  0 dt ]
  // [ 0  0  1  0 ]
  // [ 0  0  0  1 ]
  //It will be intialized to identity and dt will be updated on each time step
  F_ = Eigen::Matrix4d::Identity();
  
  // Noise matrix G:
  // [ dt^2/2       0 ]
  // [     0   dt^2/2 ]
  // [    dt        0https://github.com/jeremy-shannon/CarND-Extended-Kalman-Filter-Project.githttps://github.com/jeremy-shannon/CarND-Extended-Kalman-Filter-Project.git ]
  // [     0       dt ]
  G_ = Eigen::Matrix<double, 4, 2>::Zero();

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0; 

  Qv_ <<  9,  0,
          0,  9;

}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &mp) {

  double dt = (double)(mp.timestamp - previous_timestamp_) * 1e-6;
  previous_timestamp_ = mp.timestamp;

  if(fabs(dt) > 1000.0){ //sanity check if the sim restarts
    is_initialized_ = false;
    meas_num_ = 0;
  }
  
  meas_num_++;

  if (!is_initialized_) {
    Initialize(mp);    
    return;
  }

  Predict(dt);

  if (mp.sensor_type == MeasurementPackage::SensorType::RADAR) {
    Eigen::Vector3d z;
    z << mp.raw_measurements(0), mp.raw_measurements(1), mp.raw_measurements(2);
    RadarUpdate(z);

  } else {
    Eigen::Vector2d z;
    z << mp.raw_measurements(0), mp.raw_measurements(1);
    LidarUpdate(z);
  }

  // print the output
  std::cout << "x_ = " << x_.transpose() << std::endl;
  // std::cout << "P_ = " << P_ << std::endl << std::endl << std::endl <<std::endl;
  std::cout << "****************************************************\n";


}

Eigen::Vector4d FusionEKF::CurrentState() const{
  return x_;
}

void FusionEKF::Initialize(const MeasurementPackage& mp){
  
  if (mp.sensor_type == MeasurementPackage::SensorType::RADAR) {
    
    double rho = mp.raw_measurements(0);
    double phi = mp.raw_measurements(1);
    double rho_dot = mp.raw_measurements(2);
    x_(0) = rho * cos(phi);
    x_(1) = rho * sin(phi);
    x_(2) = rho_dot * cos(phi);
    x_(3) = rho_dot * sin(phi);
    std::cout << "Using Radar to set initial x to: " << x_.transpose() << std::endl;
  }
  else if (mp.sensor_type == MeasurementPackage::SensorType::LASER) {
    x_(0) = mp.raw_measurements(0);
    x_(1) = mp.raw_measurements(1); 
    x_(2) = 1;
    x_(3) = 1;
    std::cout << "Using LIDAR to set initial x to: " << x_.transpose() << std::endl;
  }
  previous_timestamp_ = mp.timestamp;
  // done initializing, no need to predict or update
  is_initialized_ = true;
  return;

}

void FusionEKF::Predict(double dt){
  
  F_(0, 2) = dt;
  F_(1, 3) = dt;
  G_(0, 0) = dt*dt/2.0;
  G_(1, 1) = dt*dt/2.0;
  G_(2, 0) = dt;
  G_(3, 1) = dt;

  x_ = F_ * x_;
  P_ = (F_ * P_ * F_.transpose() + G_* Qv_ * G_.transpose()).eval();
  
  std::cout << "Prediction\ndt: " << dt << " x_pred: " << x_.transpose() << std::endl << std::endl;
}

void FusionEKF::LidarUpdate(const Eigen::Vector2d& z){
 Eigen::Vector2d y = z - H_laser_ * x_;
 Eigen::Matrix2d S = H_laser_ * P_ * H_laser_.transpose() + R_laser_;
 Eigen::Matrix<double,4,2> K = P_ * H_laser_.transpose() * S.inverse();
 x_ = x_ + K * y;
 P_ = ((Eigen::Matrix4d::Identity() - K * H_laser_) * P_).eval();
 std::cout << "LIDAR measurement applied. Innovation: " << y.transpose() << "\n\n\n";
}

void FusionEKF::RadarUpdate(const Eigen::Vector3d& z){

  Eigen::Matrix<double, 3, 4> Hj = UpdateRadarJacobian();
  double rho_pred = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double phi_pred = atan2(x_(1), x_(0));
  double rho_dot_pred;
  if(fabs(rho_pred) < 0.001){
    rho_dot_pred = 0;
  } 
  else {
    rho_dot_pred = (x_(0)*x_(2) + x_(1)*x_(3))/rho_pred;
  }
  
  Eigen::Vector3d z_pred = (Eigen::Vector3d() << rho_pred, phi_pred, rho_dot_pred).finished();
  Eigen::Vector3d y = z - z_pred;

  y(1) = unwrap(y(1)); //make sure phi's innovation is between -pi and pi

  Eigen::Matrix3d S = Hj * P_ * Hj.transpose() + R_radar_; 
  Eigen::Matrix<double, 4, 3> K = P_ * Hj.transpose() * S.inverse();
  x_ = x_+ K * y;
  P_ = ((Eigen::Matrix4d::Identity() - K * Hj) * P_).eval();
  std::cout << "RADAR measurement applied. Innovation: " << y.transpose() << "\n\n\n"; 

}

Eigen::Matrix<double, 3, 4> FusionEKF::UpdateRadarJacobian(){
  
  static Eigen::Matrix<double, 3, 4> Hj = Eigen::Matrix<double, 3, 4>::Zero();
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double px2 = px * px;
  double py2 = py * py;
  double sum_sq = px2+py2; // c1
  double sqrt_sum_sq = sqrt(sum_sq);
  double pow32_sum_sq = std::pow(sum_sq, 1.5);

  // check division by zero
  if (sum_sq < 1e-6){
    std::cout << "division by zero!\n";
    return Hj;
  }
  
  // compute the Jacobian matrix
  Hj(0,0) = px/sqrt_sum_sq;
  Hj(0,1) = py/sqrt_sum_sq;
  Hj(1,0) = -py/sum_sq;
  Hj(1,1) = px/sum_sq;
  Hj(2,0) = py*(vx*py - vy*px) / pow32_sum_sq;
  Hj(2,1) = px*(vy*px - vx*py) / pow32_sum_sq;
  Hj(2,2) = px/sqrt_sum_sq;
  Hj(2,3) = py/sqrt_sum_sq;

  return Hj;
}