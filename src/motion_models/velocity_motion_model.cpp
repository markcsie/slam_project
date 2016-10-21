#include "motion_models/velocity_motion_model.h"

#include <iostream>

VelocityMotionModel::VelocityMotionModel()
{
  type_ = "Velocity Motion Model";
  dim_ = 2;
}

VelocityMotionModel::VelocityMotionModel(const VelocityMotionModel& other)
{

}

VelocityMotionModel::~VelocityMotionModel()
{

}

Eigen::VectorXd VelocityMotionModel::predictPose(const RobotModelInterface * robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    return predictPose(static_cast<const MobileRobot2dModel *> (robot_model), x, u);
  }

  std::cerr << "VelocityMotionModel::predictPose ERROR" << std::endl;
}

Eigen::VectorXd VelocityMotionModel::samplePose(const RobotModelInterface * robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    return predictPose(static_cast<const MobileRobot2dModel *> (robot_model), x, u);
  }
  std::cerr << "VelocityMotionModel::samplePose ERROR" << std::endl;
}

Eigen::VectorXd VelocityMotionModel::predictPose(const MobileRobot2dModel * robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
{
  // TODO:
  // g(x_{t-1}, u_t)
  Eigen::VectorXd x_next;
  return x_next;
}

Eigen::VectorXd VelocityMotionModel::samplePose(const MobileRobot2dModel * robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
{
  // TODO:
  // x_t ~ p(x_t| x_{t-1}, u_t)
  Eigen::VectorXd x_next;
  return x_next;
}