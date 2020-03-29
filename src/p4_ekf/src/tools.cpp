#include "tools.h"
#include <iostream>

Eigen::Vector4d CalculateRMSE(const std::vector<Eigen::Vector4d> &estimations,
                              const std::vector<Eigen::Vector4d> &ground_truth) {

   Eigen::Vector4d rmse = Eigen::Vector4d::Zero();                              
   if(estimations.size() != ground_truth.size()){
      std::cout << "Input vectors must be same length!\n";
      return rmse;
   }
   
   size_t n = estimations.size();
   for (size_t i = 0; i < n; ++i){
      Eigen::Vector4d residual = estimations[i] - ground_truth[i]; 
      residual = residual.array() * residual.array();
      rmse +=residual;
   }

   rmse = rmse / n;

   rmse = rmse.array().sqrt();

   return rmse;
}

//adjust an angle so that -pi < angle < pi
double unwrap(double angle){

   while(angle > kPI){
      angle -= k2PI;
   }

   while(angle < -kPI){
      angle += k2PI;
   }
   return angle;
}