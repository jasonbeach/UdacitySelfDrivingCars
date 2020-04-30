#pragma once

#include <string>
#include <vector>
#include <random>
#include "Eigen/Dense"
#include "helper_functions.h"
#include "map.h"


struct Particle{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double theta = 0;
  double weight = 0;
  std::vector<int> last_associated_landmarks;
  std::string GetLandmarks() const;
};

struct SensorObservation{ 
  uint64_t timestamp = 0; //not used for now;
  Eigen::Vector3d data = Eigen::Vector3d::Zero();// 2-D position in homogeneous coordinates
  std::string frame;
};

using SensorObservationList = std::vector<SensorObservation>;

struct AssociatedSensorObservation{
  SensorObservation obs;
  Landmark landmark;
};

using AssociatedSensorObservationList = std::vector<AssociatedSensorObservation>;

struct ParticleFilterParams{
  Map map;
  std::array<double, 3> sigma_pos = {};
  std::array<double, 2> sigma_sense = {};
  Particle filter_seed = {};
  size_t num_particles = 0;
  double max_sensor_range = 0;
};

class ParticleFilter{
public:
  ParticleFilter() = default;
  ParticleFilter(const ParticleFilterParams& p);

  void Predict(double dt, double speed, double yaw_rate);

  void SensorUpdate(const SensorObservationList& obs);

  operator bool() const;

  Particle GetBestParticle() const;

  double GetParticleAverageWeight() const;

private:
  SensorObservationList ConvertSensorObservations(const Particle& p, 
    const SensorObservationList& obs_list_car) const;

  SensorObservation ToMapCoords(double cos_theta, double sin_theta, 
    const Eigen::Vector3d& p_pos, const SensorObservation& obs_car) const;
  // DataAssociation
  void UpdateWeights(const SensorObservationList& obs_list_car);
  Particle UpdateParticleWeight(Particle const&, SensorObservationList const& obs_list_map);
  void Resample();

  ParticleFilterParams params_;
  std::vector<Particle> particles_;
  std::default_random_engine gen_;
  
  std::normal_distribution<double> dist_x_;
  std::normal_distribution<double> dist_y_;
  std::normal_distribution<double> dist_theta_;
};
