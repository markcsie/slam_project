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

Eigen::MatrixXd VelocityMotionModel::calculateRt(const std::shared_ptr<const RobotModelInterface> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u, const Eigen::MatrixXd &cov) const
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    return calculateRt(std::static_pointer_cast<const MobileRobot2dModel>(robot_model), x, u, cov);
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
    return samplePose(std::static_pointer_cast<const MobileRobot2dModel>(robot_model), x, u);
  }
  std::cerr << "VelocityMotionModel::samplePose ERROR" << std::endl;
}

Eigen::MatrixXd VelocityMotionModel::calculateRt(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u, const Eigen::MatrixXd &cov) const
{
//  std::cout << "ggg u[1] " << u[1] << std::endl;
  
  Eigen::MatrixXd G_t(robot_model->getDim(), robot_model->getDim());
  if (u[1] == 0)
  {
    G_t << 1, 0, -u[0] * std::sin(x[2]) * delta_t_,
            0, 1, u[0] * std::cos(x[2]) * delta_t_,
            0, 0, 1;
  }
  else
  {
    double r = u[0] / u[1];
    G_t << 1, 0, -r * std::cos(x[2]) + r * std::cos(x[2] + u[1] * delta_t_),
            0, 1, -r * std::sin(x[2]) + r * std::sin(x[2] + u[1] * delta_t_),
            0, 0, 1;
  }

  Eigen::MatrixXd V_t(robot_model->getDim(), dim_);
  if (u[1] == 0)
  {
    V_t << std::cos(x[2]) * delta_t_, 0,
           std::sin(x[2]) * delta_t_, 0,
            0, 0;
  }
  else
  {
    V_t(0, 0) = (-std::sin(x[2]) + std::sin(x[2] + u[1] * delta_t_)) / u[1];
    V_t(0, 1) = u[0] * ((std::sin(x[2]) - std::sin(x[2] + u[1] * delta_t_)) / std::pow(u[1], 2) + (std::cos(x[2] + u[1] * delta_t_) * delta_t_) / u[1]);
    V_t(1, 0) = (std::cos(x[2]) - std::cos(x[2] + u[1] * delta_t_)) / u[1];
    V_t(1, 1) = -u[0] * ((std::cos(x[2]) - std::cos(x[2] + u[1] * delta_t_)) / std::pow(u[1], 2) - (std::sin(x[2] + u[1] * delta_t_) * delta_t_) / u[1]);
    V_t(2, 0) = 0;
    V_t(2, 1) = delta_t_;
  }

//  std::cout << "ggg V_t \n" << V_t << std::endl;

  Eigen::MatrixXd M_t(dim_, dim_);
  M_t << alphas_[0] * std::pow(u[0], 2) + alphas_[1] * std::pow(u[1], 2), 0,
          0, alphas_[2] * std::pow(u[0], 2) + alphas_[3] * std::pow(u[1], 2);
  
//  std::cout << "ggg M_t" << std::endl << M_t << std::endl;
//  std::cout << "ggg cov" << std::endl << cov << std::endl;
  
  return G_t * cov * G_t.transpose() + V_t * M_t * V_t.transpose();
};

Eigen::VectorXd VelocityMotionModel::predictPose(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  // g(x_{t-1}, u_t)
  Eigen::VectorXd x_next(robot_model->getDim());
  if (u[1] == 0)
  {
    x_next << x[0] + u[0] * std::cos(x[2]) * delta_t_, x[1] + u[0] * std::sin(x[2]) * delta_t_, x[2];
  }
  else
  {
    double r = u[0] / u[1];
    x_next << x[0] - r * std::sin(x[2]) + r * std::sin(x[2] + u[1] * delta_t_), x[1] + r * std::cos(x[2]) - r * std::cos(x[2] + u[1] * delta_t_), x[2] + u[1] * delta_t_;
  }
  return x_next;
}

Eigen::VectorXd VelocityMotionModel::samplePose(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  // x_t ~ p(x_t| x_{t-1}, u_t)
  Eigen::VectorXd x_next(robot_model->getDim());
//  std::cout << "ggg u " << u.transpose() << std::endl;
  double v_hat = u[0] + Utils::sampleGaussian(0, alphas_[0] * std::pow(u[0], 2) + alphas_[1] * std::pow(u[1], 2));
  double w_hat = u[1] + Utils::sampleGaussian(0, alphas_[2] * std::pow(u[0], 2) + alphas_[3] * std::pow(u[1], 2));
  double gamma_hat = Utils::sampleGaussian(0, alphas_[4] * std::pow(u[0], 2) + alphas_[5] * std::pow(u[1], 2));
//  std::cout << "ggg v_hat " << v_hat << std::endl;
//  std::cout << "ggg w_hat " << w_hat << std::endl;
//  std::cout << "ggg gamma_hat " << gamma_hat << std::endl;
  if (w_hat == 0)
  {
    x_next << x[0] + v_hat * std::cos(x[2]) * delta_t_, x[1] + v_hat * std::sin(x[2]) * delta_t_, x[2] + gamma_hat * delta_t_;
  }
  else
  {
    double r = v_hat / w_hat;
    x_next << x[0] - r * std::sin(x[2]) + r * std::sin(x[2] + w_hat * delta_t_), x[1] + r * std::cos(x[2]) - r * std::cos(x[2] + w_hat * delta_t_), x[2] + w_hat * delta_t_ + gamma_hat * delta_t_;
  }
  return x_next;
}