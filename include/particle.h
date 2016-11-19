#ifndef PARTICLE_H
#define PARTICLE_H

#include "utils/gaussian.h"

struct Particle
{
  Eigen::VectorXd x_;
  Eigen::MatrixXd cov_; // Cov of proposal distribution
  std::unordered_map<int, Gaussian> features_; // key = subject # value Gaussian(mean, covariance)
};

struct MultiRobotParticle
{
  std::unordered_map<int, Eigen::VectorXd> x_;
  Eigen::MatrixXd cov_; // Cov of proposal distribution
  std::unordered_map<int, Gaussian> features_; // key = subject # value Gaussian(mean, covariance)
};

#endif /* PARTICLE_H */

