#ifndef INTERFACE_MOTION_MODEL_H
#define INTERFACE_MOTION_MODEL_H

#include <Eigen/Dense>

#include "../robot_models/interface_robot_model.h"

class RobotModelInterface;

class MotionModelInterface
{
public:
  virtual Eigen::VectorXd samplePose(const RobotModelInterface * robot_model, const Eigen::VectorXd &x, const Eigen::VectorXd &u) = 0;
  virtual Eigen::VectorXd predictPose(const RobotModelInterface * robot_model, const Eigen::VectorXd &x, const Eigen::VectorXd &u) = 0;
  
  const std::string &getType() const
  {
    return type_;
  };

  const size_t &getDim() const
  {
    return dim_;
  };

protected:
  std::string type_;
  size_t dim_;
};

#endif /* INTERFACE_MOTION_MODEL_H */

