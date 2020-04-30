#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <array>
#include <random>
#include "json.hpp"
#include "map.h"
#include "particle_filter.hpp"

// for convenience
using nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  } 
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;
  ParticleFilterParams pf_params;

  // Read map data
  MapReader map_reader{"../data/map_data.txt"};
  pf_params.map = map_reader.GetMap();
  
  // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  pf_params.sigma_pos = {0.3, 0.3, 0.01};

  // Landmark measurement uncertainty [x [m], y [m]]
  pf_params.sigma_sense = {0.3, 0.3};

  // number of particles
  pf_params.num_particles = 25;

  // max sensor range [m]
  pf_params.max_sensor_range = 50.0;

  // Create particle filter - default constructed for now
  ParticleFilter pf;

  // this really isn't a param as it's not normally constant, but for this we'll say it is.
  double delta_t = 0.1;  // Time elapsed between measurements [sec]
  

  h.onMessage([&pf, &pf_params, &delta_t]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    (void) opCode;
        
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data));

      if (s != "") {
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {

          // j[1] is the data JSON object
          if (!pf) { //check for default constructed particle filter
            // Sense noisy position data from the simulator
            pf_params.filter_seed.position.x() = std::stod(j[1]["sense_x"].get<std::string>());
            pf_params.filter_seed.position.y() = std::stod(j[1]["sense_y"].get<std::string>());
            pf_params.filter_seed.position.z() = 1.0; 
            pf_params.filter_seed.theta = std::stod(j[1]["sense_theta"].get<std::string>());
            pf_params.filter_seed.weight = 1.0;
            pf = ParticleFilter(pf_params);
          } else {
            // Predict the vehicle's next state from previous 
            //   (noiseless control) data.

            double previous_speed = std::stod(j[1]["previous_velocity"].get<std::string>()); // velocity is a vector. This is not.
            double previous_yawrate = std::stod(j[1]["previous_yawrate"].get<std::string>());

            pf.Predict(delta_t, previous_speed, previous_yawrate);
          }

          // receive noisy observation data from the simulator
          // sense_observations in JSON format 
          //   [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}] <- the code below suggests otherwise - JB
          SensorObservationList noisy_observations;
          std::string sense_observations_x = j[1]["sense_observations_x"];
          std::string sense_observations_y = j[1]["sense_observations_y"];

          std::istringstream iss_x(sense_observations_x);
          std::istringstream iss_y(sense_observations_y);

          std::transform(std::istream_iterator<float>(iss_x), std::istream_iterator<float>(), 
            std::istream_iterator<float>(iss_y), std::back_inserter(noisy_observations), 
            [](float x, float y)-> SensorObservation {
              SensorObservation so; 
              so.timestamp = 0;
              so.data << x, y, 1.0;
              so.frame = "car_frame";
              return so;  
            });
          
          // Process Sensor Observations
          pf.SensorUpdate(noisy_observations);

          // Get best particle and average particle weight

          Particle best_particle = pf.GetBestParticle();
          double average_weight = pf.GetParticleAverageWeight();

          std::cout << "highest w " << best_particle.weight << std::endl;
          std::cout << "average w " << average_weight << std::endl;

          json msgJson;
          msgJson["best_particle_x"] = best_particle.position.x();
          msgJson["best_particle_y"] = best_particle.position.y();
          msgJson["best_particle_theta"] = best_particle.theta;

          // Optional message data used for debugging particle's sensing 
          //   and associations
          msgJson["best_particle_associations"] = best_particle.GetLandmarks();
          msgJson["best_particle_sense_x"] = ""; //pf.getSenseCoord(best_particle, "X");
          msgJson["best_particle_sense_y"] = ""; //pf.getSenseCoord(best_particle, "Y");

          auto msg = "42[\"best_particle\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    (void) ws;
    (void) req;
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    (void) code;
    (void) message;
    (void) length;
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}