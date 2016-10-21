#ifndef GAUSSIAN_H
#define GAUSSIAN_H

static std::default_random_engine generator;

namespace Utils
{

  double sampleGaussian(const double &mean, const double &variance)
  {
    std::normal_distribution<double> distribution(mean, variance);
    return distribution(generator);
  }
}

#endif /* GAUSSIAN_H */

