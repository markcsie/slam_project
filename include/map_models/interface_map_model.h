#ifndef INTERFACE_MAP_MODEL_H
#define INTERFACE_MAP_MODEL_H

#include <string>
#include <Eigen/Dense>

#include "../utils/random_utils.h"

class MapModelInterface
{
public:
  virtual const std::string &getType() const
  {
    return type_;
  };

  virtual const size_t &getDim() const
  {
    return dim_;
  };
  
  virtual Eigen::VectorXd getRandomPosition() const
  {
    Eigen::VectorXd p(dim_);
    for (size_t i = 0; i < p.rows(); i++)
    {
      p[i] = corner_[i] + size_[i] * Utils::sampleUniform(0.0, 1.0);
    }
    return p;
  };
  
protected:
  std::string type_;
  size_t dim_;
  Eigen::VectorXd size_;
  Eigen::VectorXd corner_;
};

#endif /* INTERFACE_MAP_MODEL_H */

