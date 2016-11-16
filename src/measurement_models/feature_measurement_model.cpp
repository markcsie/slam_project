#include "measurement_models/feature_measurement_model.h"

#include <iostream>

FeatureMeasurementModel::FeatureMeasurementModel(const Eigen::MatrixXd &Q_t) : Q_t_(Q_t)
{
  type_ = "Feature Measurement Model";
  dim_ = 2;
  assert(Q_t_.rows() == 2 && Q_t_.cols() == 2);
}

FeatureMeasurementModel::FeatureMeasurementModel(const FeatureMeasurementModel& other)
{

}

FeatureMeasurementModel::~FeatureMeasurementModel()
{
}

Eigen::VectorXd FeatureMeasurementModel::predictMeasurement(const std::shared_ptr<const RobotModelInterface> &robot_model, const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    if (map_model->getType() == FeatureMap2dModel::TYPE)
    {
      return predictMeasurement(std::static_pointer_cast<const MobileRobot2dModel>(robot_model), std::static_pointer_cast<const FeatureMap2dModel>(map_model), mean, x);
    }
  }
  std::cerr << "FastSlam2::predictMeasurement ERROR" << std::endl;
}

Eigen::VectorXd FeatureMeasurementModel::inverseMeasurement(const std::shared_ptr<const RobotModelInterface> &robot_model, const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd& x, const Eigen::VectorXd& z) const
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    if (map_model->getType() == FeatureMap2dModel::TYPE)
    {
      return inverseMeasurement(std::static_pointer_cast<const MobileRobot2dModel>(robot_model), std::static_pointer_cast<const FeatureMap2dModel>(map_model), x, z);
    }
  }
  std::cerr << "FastSlam2::inverseMeasurement ERROR" << std::endl;
}

Eigen::MatrixXd FeatureMeasurementModel::jacobianPose(const std::shared_ptr<const RobotModelInterface> &robot_model, const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    if (map_model->getType() == FeatureMap2dModel::TYPE)
    {
      return jacobianPose(std::static_pointer_cast<const MobileRobot2dModel>(robot_model), std::static_pointer_cast<const FeatureMap2dModel>(map_model), mean, x);
    }
  }
  std::cerr << "FastSlam2::jacobianPose ERROR" << std::endl;
}

Eigen::MatrixXd FeatureMeasurementModel::jacobianFeature(const std::shared_ptr<const RobotModelInterface> &robot_model, const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const
{
  if (robot_model->getType() == MobileRobot2dModel::TYPE)
  {
    if (map_model->getType() == FeatureMap2dModel::TYPE)
    {
      return jacobianFeature(std::static_pointer_cast<const MobileRobot2dModel>(robot_model), std::static_pointer_cast<const FeatureMap2dModel>(map_model), mean, x);
    }
  }
  std::cerr << "FastSlam2::jacobianFeature ERROR" << std::endl;
}

Eigen::VectorXd FeatureMeasurementModel::predictMeasurement(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const std::shared_ptr<const FeatureMap2dModel> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const
{
  // z = h(mean_{t-1}, x)
  Eigen::VectorXd z(dim_);
  z << std::sqrt(std::pow(mean[0] - x[0], 2) + std::pow(mean[1] - x[1], 2)), std::atan2(mean[1] - x[1], mean[0] - x[0]) - x[2];
  return z;
}

Eigen::VectorXd FeatureMeasurementModel::inverseMeasurement(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const std::shared_ptr<const FeatureMap2dModel> &map_model, const Eigen::VectorXd& x, const Eigen::VectorXd& z) const
{
  // mean_t = h^{-1}(x_t, z_t))
  Eigen::VectorXd mean(map_model->getDim());
  mean << x[0] + z[0] * std::cos(z[1] + x[2]), x[1] + z[0] * std::sin(z[1] + x[2]);
  return mean;
}

Eigen::MatrixXd FeatureMeasurementModel::jacobianPose(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const std::shared_ptr<const FeatureMap2dModel> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const
{
  Eigen::MatrixXd H_x(dim_, robot_model->getDim());
  double q = std::pow(mean[0] - x[0], 2) + std::pow(mean[1] - x[1], 2);
//  std::cout << "ggg mean " << mean.transpose() << std::endl;
//  std::cout << "ggg x " << x.transpose() << std::endl;
  assert(q > 0);
  H_x << -(mean[0] - x[0]) / std::sqrt(q), -(mean[1] - x[1]) / std::sqrt(q), 0,
          (mean[1] - x[1]) / q, -(mean[0] - x[0]) / q, -1;
  return H_x;
}

Eigen::MatrixXd FeatureMeasurementModel::jacobianFeature(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const std::shared_ptr<const FeatureMap2dModel> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const
{
  Eigen::MatrixXd H_m(dim_, map_model->getDim());
  double q = std::pow(mean[0] - x[0], 2) + std::pow(mean[1] - x[1], 2);
//  std::cout << "ggg mean " << mean.transpose() << std::endl;
//  std::cout << "ggg x " << x.transpose() << std::endl;
//  std::cout << "ggg q " << q << std::endl;
  assert(q > 0);
  H_m << (mean[0] - x[0]) / std::sqrt(q), (mean[1] - x[1]) / std::sqrt(q),
          -(mean[1] - x[1]) / q, (mean[0] - x[0]) / q;
  return H_m;
}
