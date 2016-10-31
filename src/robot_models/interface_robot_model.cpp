#include "robot_models/interface_robot_model.h"

RobotModelInterface::RobotModelInterface()
{

}

RobotModelInterface::~RobotModelInterface()
{
}

const Eigen::MatrixXd &RobotModelInterface::getQt() const
{
  return measurement_model_->getQt();
};

Eigen::MatrixXd RobotModelInterface::calculateRt(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const
{
  return motion_model_->calculateRt(this->shared_from_this(), x, u);
};

Eigen::VectorXd RobotModelInterface::predictPose(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  return motion_model_->predictPose(this->shared_from_this(), x, u);
}

Eigen::VectorXd RobotModelInterface::samplePose(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  return motion_model_->samplePose(this->shared_from_this(), x, u);
}

Eigen::VectorXd RobotModelInterface::predictMeasurement(const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const
{
  return measurement_model_->predictMeasurement(this->shared_from_this(), map_model, mean, x);
}

Eigen::VectorXd RobotModelInterface::inverseMeasurement(const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd& x, const Eigen::VectorXd& z) const
{
  return measurement_model_->inverseMeasurement(this->shared_from_this(), map_model, x, z);
}

Eigen::MatrixXd RobotModelInterface::jacobianPose(const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const
{
  return measurement_model_->jacobianPose(this->shared_from_this(), map_model, mean, x);
}

Eigen::MatrixXd RobotModelInterface::jacobianFeature(const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const
{
  return measurement_model_->jacobianFeature(this->shared_from_this(), map_model, mean, x);
}