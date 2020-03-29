#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

constexpr double k2PI = 2.0 * M_PI;
constexpr double kPI = M_PI;

Eigen::Vector4d CalculateRMSE(const std::vector<Eigen::Vector4d> &estimations, 
                                const std::vector<Eigen::Vector4d> &ground_truth);

double unwrap(double angle);

#endif  // TOOLS_H_
