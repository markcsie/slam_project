#ifndef RANDOM_UTILS_H
#define RANDOM_UTILS_H

#include "eigenmvn.h"

static std::default_random_engine generator;

namespace Utils
{

  inline double sampleGaussian(const double &mean, const double &variance)
  {
    std::normal_distribution<double> distribution(mean, std::sqrt(variance));
    return distribution(generator);
  }

  inline double sampleUniform(const double &lower_bound, const double &upper_bound)
  {
    std::uniform_real_distribution<double> distribution(lower_bound, upper_bound);
    return distribution(generator);
  }

  inline size_t sampleDiscrete(const std::vector<double> &weights)
  {
    std::discrete_distribution<size_t> distribution(weights.begin(), weights.end());
    return distribution(generator);
  }

  inline Eigen::VectorXd sampleMultivariateGaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
  {
    // x ~ N(mean, covariance)
    Eigen::EigenMultivariateNormal<double> norm(mean, covariance);
    return norm.samples(1);
  }
  
}

#endif /* RANDOM_UTILS_H */

