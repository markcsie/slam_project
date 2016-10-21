#ifndef FEATURE_MEASUREMENT_MODEL_H
#define FEATURE_MEASUREMENT_MODEL_H

#include "interface_measurement_model.h"

#include "../robot_models/mobile_robot_2dmodel.h"
#include "../map_models/feature_map_2dmodel.h"

class FeatureMeasurementModel : public MeasurementModelInterface
{
public:
  FeatureMeasurementModel(const Eigen::MatrixXd &Q_t);
  FeatureMeasurementModel(const FeatureMeasurementModel& other);
  virtual ~FeatureMeasurementModel();

  virtual Eigen::VectorXd predictMeasurement(const std::shared_ptr<const RobotModelInterface> &robot_model, const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;
  virtual Eigen::VectorXd inverseMeasurement(const std::shared_ptr<const RobotModelInterface> &robot_model, const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd &x, const Eigen::VectorXd &z) const;
  virtual Eigen::MatrixXd jacobianPose(const std::shared_ptr<const RobotModelInterface> &robot_model, const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const;
  virtual Eigen::MatrixXd jacobianFeature(const std::shared_ptr<const RobotModelInterface> &robot_model, const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;

  const Eigen::MatrixXd &getQt() const
  {
    return Q_t_;
  };
  
private:
  virtual Eigen::VectorXd predictMeasurement(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const std::shared_ptr<const FeatureMap2dModel> &map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;
  virtual Eigen::VectorXd inverseMeasurement(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const std::shared_ptr<const FeatureMap2dModel> &map_model, const Eigen::VectorXd &x, const Eigen::VectorXd &z) const;
  virtual Eigen::MatrixXd jacobianPose(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const std::shared_ptr<const FeatureMap2dModel> &map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) const;
  virtual Eigen::MatrixXd jacobianFeature(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const std::shared_ptr<const FeatureMap2dModel> &map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;

  Eigen::MatrixXd Q_t_;

};

#endif /* FEATURE_MEASUREMENT_MODEL_H */

