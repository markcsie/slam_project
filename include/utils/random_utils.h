#ifndef RANDOM_UTILS_H
#define RANDOM_UTILS_H

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
  
}

#endif /* RANDOM_UTILS_H */

