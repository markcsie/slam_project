#include "motion_models/velocity_motion_model.h"

#include <iostream>

#include "utils/random_utils.h"

VelocityMotionModel::VelocityMotionModel(const std::vector<double> &alphas, const double &delta_t) : alphas_(alphas), delta_t_(delta_t)
{
  type_ = "Velocity Motion Model";
  dim_ = 2;
  assert(alphas_.size() == 6);
}

VelocityMotionModel::VelocityMotionModel(const VelocityMotionModel& other)
{

}

VelocityMotionModel::~VelocityMotionModel()
{
}

Eigen::MatrixXd VelocityMotionModel::calculateRt(const std::shared_ptr<const RobotModelInterface> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    return calculateRt(std::static_pointer_cast<const MobileRobot2dModel>(robot_model), x, u);
  }
  std::cerr << "VelocityMotionModel::calculateRt ERROR" << std::endl;
};

Eigen::VectorXd VelocityMotionModel::predictPose(const std::shared_ptr<const RobotModelInterface> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    return predictPose(std::static_pointer_cast<const MobileRobot2dModel>(robot_model), x, u);
  }
  std::cerr << "VelocityMotionModel::predictPose ERROR" << std::endl;
}

Eigen::VectorXd VelocityMotionModel::samplePose(const std::shared_ptr<const RobotModelInterface> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    return predictPose(std::static_pointer_cast<const MobileRobot2dModel>(robot_model), x, u);
  }
  std::cerr << "VelocityMotionModel::samplePose ERROR" << std::endl;
}

Eigen::MatrixXd VelocityMotionModel::calculateRt(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  Eigen::MatrixXd V_t(robot_model->getDim(), dim_);
  V_t(0, 0) = (-std::sin(x[2]) + std::sin(x[2] + u[1] * delta_t_)) / u[1];
  V_t(0, 1) = u[0] * ((std::sin(x[2]) - std::sin(x[2] + u[1] * delta_t_)) / std::pow(u[1], 2) + (std::cos(x[2] + u[1] * delta_t_) * delta_t_) / u[1]);
  V_t(1, 0) = (std::cos(x[2]) - std::cos(x[2] + u[1] * delta_t_)) / u[1];
  V_t(1, 1) = -u[0] * ((std::cos(x[2]) - std::cos(x[2] + u[1] * delta_t_)) / std::pow(u[1], 2) - (std::sin(x[2] + u[1] * delta_t_) * delta_t_) / u[1]);
  V_t(2, 0) = 0;
  V_t(2, 1) = delta_t_;
  
  Eigen::MatrixXd M_t(dim_, dim_);
  M_t << alphas_[0] * std::pow(u[0], 2) + alphas_[1] * std::pow(u[1], 2), 0,
         0, alphas_[2] * std::pow(u[0], 2) + alphas_[3] * std::pow(u[1], 2);
  return V_t * M_t * V_t.transpose();
};

Eigen::VectorXd VelocityMotionModel::predictPose(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  // g(x_{t-1}, u_t)
  Eigen::VectorXd x_next(robot_model->getDim());
  double q = u[0] / u[1];
  x_next << x[0] - q * std::sin(x[2]) + q * std::sin(x[2] + u[1] * delta_t_), x[1] + q * std::cos(x[2]) - q * std::cos(x[2] + u[1] * delta_t_), x[2] + u[1] * delta_t_;
  return x_next;
}

Eigen::VectorXd VelocityMotionModel::samplePose(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  // x_t ~ p(x_t| x_{t-1}, u_t)
  Eigen::VectorXd x_next(robot_model->getDim());
  
  double v_hat = u[0] + Utils::sampleGaussian(0, alphas_[0] * std::pow(u[0], 2) + alphas_[1] * std::pow(u[1], 2));
  double w_hat = u[1] + Utils::sampleGaussian(0, alphas_[2] * std::pow(u[0], 2) + alphas_[3] * std::pow(u[1], 2));
  double gamma_hat = Utils::sampleGaussian(0, alphas_[4] * std::pow(u[0], 2) + alphas_[5] * std::pow(u[1], 2));
  double q = v_hat / w_hat;
  x_next << x[0] - q * std::sin(x[2]) + q * std::sin(x[2] + w_hat * delta_t_), x[1] + q * std::cos(x[2]) - q * std::cos(x[2] + w_hat * delta_t_), x[2] + w_hat * delta_t_ + gamma_hat * delta_t_;
  return x_next;
}