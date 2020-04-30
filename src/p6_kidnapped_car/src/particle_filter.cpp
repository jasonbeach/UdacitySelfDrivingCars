#include "particle_filter.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iterator>

std::string Particle::GetLandmarks() const{
  std::stringstream ss;
  copy(last_associated_landmarks.begin(), last_associated_landmarks.end(), std::ostream_iterator<int>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

ParticleFilter::ParticleFilter(const ParticleFilterParams& p): params_{p}, 
  dist_x_(0, p.sigma_pos[0]), dist_y_(0, p.sigma_pos[1]), dist_theta_(0, p.sigma_pos[2]) {
  
  std::cout << "Constructing PF seed position: " << p.filter_seed.position.x() << ", " 
    << p.filter_seed.position.y() << ", theta: " << p.filter_seed.theta * kR2D << " deg \n";
  std::generate_n(std::back_inserter(particles_), p.num_particles, 
    [&](){
    Particle np;    
    np.position.x() = p.filter_seed.position.x() + dist_x_(gen_);
    np.position.y() = p.filter_seed.position.y() + dist_y_(gen_);
    np.position.z() = 1.0;
    np.theta = wrap2pi(p.filter_seed.theta + dist_theta_(gen_));
    np.weight = 1.0;
    return np;
  });
}

void ParticleFilter::Predict(double dt, double speed, double theta_dot){

  std::for_each(particles_.begin(), particles_.end(),
    [&](Particle& p){
    if(std::abs(theta_dot) < 1e-6){ // for tiny yaw rates
      p.position.x() += speed * dt * cos(p.theta) + dist_x_(gen_);
      p.position.y() += speed * dt * sin(p.theta) + dist_y_(gen_);
      p.theta = wrap2pi( p.theta + dist_theta_(gen_ ));
    }
    else{
      p.position.x() += speed / theta_dot * (sin(p.theta + theta_dot * dt) - sin(p.theta)) + dist_x_(gen_);
      p.position.y() += speed / theta_dot * (cos(p.theta) - cos(p.theta + theta_dot * dt)) + dist_y_(gen_);
      p.theta  = wrap2pi( p.theta + theta_dot * dt + dist_theta_(gen_));
    }
  });
}

void ParticleFilter::SensorUpdate(const SensorObservationList& obs_list_car){
  UpdateWeights(obs_list_car);
  Resample();
}

ParticleFilter::operator bool() const{
 return !particles_.empty();
}

Particle ParticleFilter::GetBestParticle() const{       
  return *std::max_element(particles_.begin(), particles_.end(), 
    [](const Particle& p1, const Particle& p2){return p1.weight < p2.weight;});
}

double ParticleFilter::GetParticleAverageWeight() const{
  double weight_sum = std::accumulate(particles_.begin(), particles_.end(), 0.0, 
    [](double w_sum, const Particle& p) {return w_sum + p.weight;} );
  return weight_sum / particles_.size();
}

SensorObservationList ParticleFilter::ConvertSensorObservations(const Particle& p, 
  const SensorObservationList& obs_list_car) const{
  double ct = std::cos(p.theta); // precompute and use for all observations
  double st = std::sin(p.theta);
 //std::random_device rd_;
  std::mt19937 rng_;
  SensorObservationList obs_list_map;
  std::transform(obs_list_car.begin(), obs_list_car.end(),std::back_inserter(obs_list_map),
    [&](const SensorObservation& obs_car)-> SensorObservation {
        return ToMapCoords(ct, st, p.position, obs_car);
      });

  return obs_list_map;
}

SensorObservation ParticleFilter::ToMapCoords(double cos_theta, double sin_theta, 
  const Eigen::Vector3d& p_pos, const SensorObservation& obs_car) const{
      Eigen::Matrix3d tf = (Eigen::Matrix3d() << 
        cos_theta, -sin_theta, p_pos.x(), 
        sin_theta,  cos_theta, p_pos.y(), 
                0,          0,      1.0).finished();

      SensorObservation obs_map;
      obs_map.data = tf * obs_car.data;
      obs_map.frame = "map_frame";
      return obs_map;
}

// DataAssociation

void ParticleFilter::UpdateWeights(const SensorObservationList& obs_list_car){
  
  std::transform(particles_.begin(), particles_.end(), particles_.begin(),
    [&](const Particle& p) {
      SensorObservationList obs_list_map = this->ConvertSensorObservations(p, obs_list_car);
      return UpdateParticleWeight(p, obs_list_map);
    });
}

Particle ParticleFilter::UpdateParticleWeight(Particle const& p, SensorObservationList const& obs_list_map){
  Particle new_particle;
  new_particle.position = p.position;
  new_particle.theta = p.theta;
  new_particle.weight = 1.0;

  const float k = 1.0 / (k2PI * params_.sigma_sense[0] * params_.sigma_sense[1]);
  const float k2sx = 2.0 * params_.sigma_sense[0] * params_.sigma_sense[0];
  const float k2sy = 2.0 * params_.sigma_sense[1] * params_.sigma_sense[1];

  std::for_each(obs_list_map.begin(), obs_list_map.end(), [&](SensorObservation const& so){
    Landmark l = params_.map.NearestLandmark(so.data, params_.max_sensor_range);

    if(l.id > 0){
      float dx = l.location.x() - so.data.x();
      float dy = l.location.y() - so.data.y();

      float p_xy = k * std::exp(-( (dx*dx/k2sx) + (dy*dy/k2sy) ) );
      new_particle.weight *= p_xy;
      new_particle.last_associated_landmarks.push_back(l.id);
    }
    else{
      new_particle.weight *= 1e-100;
    }
  });

  return new_particle;
}

void ParticleFilter::Resample(){
  Eigen::VectorXf w;
  w.resize(params_.num_particles);

  std::transform(particles_.begin(), particles_.end(), w.data(),
    [&](const Particle p) ->float
    {
      return p.weight;
    } );
  
  static std::random_device rd;
  static std::mt19937 rng(rd());

  float W = std::accumulate(w.data(), w.data() + w.size(), 0.0);
  Eigen::VectorXf alpha = w / W;

  std::discrete_distribution<int> dist(alpha.data(), alpha.data() + alpha.size());

  std::vector<Particle> new_particles;

  std::generate_n(std::back_inserter(new_particles), params_.num_particles, 
    [&](){
    int index = dist(rng);
    return particles_[index];
  });

  particles_ = new_particles;
}