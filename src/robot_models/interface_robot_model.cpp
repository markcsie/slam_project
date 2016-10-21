#include "robot_models/interface_robot_model.h"

RobotModelInterface::RobotModelInterface()
{

}

RobotModelInterface::~RobotModelInterface()
{
  if (motion_model_ != nullptr) {
    delete motion_model_;
    motion_model_ = nullptr;
  }
  if (measurement_model_ != nullptr) {
    delete measurement_model_;
    measurement_model_ = nullptr;
  }
}

const Eigen::MatrixXd &RobotModelInterface::getQt() const
{
  return measurement_model_->getQt();
};

Eigen::VectorXd RobotModelInterface::predictPose(const Eigen::VectorXd& x, const Eigen::VectorXd& u)
{
  return motion_model_->predictPose(this, x, u);
}

Eigen::VectorXd RobotModelInterface::samplePose(const Eigen::VectorXd& x, const Eigen::VectorXd& u)
{
  return motion_model_->samplePose(this, x, u);
}

Eigen::VectorXd RobotModelInterface::predictMeasurement(const MapModelInterface *map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x)
{
  return measurement_model_->predictMeasurement(this, map_model, mean, x);
}

Eigen::VectorXd RobotModelInterface::inverseMeasurement(const MapModelInterface *map_model, const Eigen::VectorXd& x, const Eigen::VectorXd& z)
{
  return measurement_model_->inverseMeasurement(this, map_model, x, z);
}

Eigen::MatrixXd RobotModelInterface::jacobianPose(const MapModelInterface *map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x)
{
  return measurement_model_->jacobianPose(this, map_model, mean, x);
}

Eigen::MatrixXd RobotModelInterface::jacobianFeature(const MapModelInterface *map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x)
{
  return measurement_model_->jacobianFeature(this, map_model, mean, x);
}