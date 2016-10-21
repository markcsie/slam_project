#ifndef FEATURE_MEASUREMENT_MODEL_H
#define FEATURE_MEASUREMENT_MODEL_H

#include "interface_measurement_model.h"

#include "../robot_models/mobile_robot_2dmodel.h"
#include "../map_models/feature_map_2dmodel.h"

class FeatureMeasurementModel : public MeasurementModelInterface
{
public:
  FeatureMeasurementModel();
  FeatureMeasurementModel(const FeatureMeasurementModel& other);
  virtual ~FeatureMeasurementModel();

  virtual Eigen::VectorXd predictMeasurement(const RobotModelInterface * robot_model, const MapModelInterface * map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
  virtual Eigen::VectorXd inverseMeasurement(const RobotModelInterface * robot_model, const MapModelInterface * map_model, const Eigen::VectorXd &x, const Eigen::VectorXd &z);
  virtual Eigen::MatrixXd jacobianPose(const RobotModelInterface * robot_model, const MapModelInterface * map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x);
  virtual Eigen::MatrixXd jacobianFeature(const RobotModelInterface * robot_model, const MapModelInterface * map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
  
private:
  virtual Eigen::VectorXd predictMeasurement(const MobileRobot2dModel * robot_model, const FeatureMap2dModel * map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
  virtual Eigen::VectorXd inverseMeasurement(const MobileRobot2dModel * robot_model, const FeatureMap2dModel * map_model, const Eigen::VectorXd &x, const Eigen::VectorXd &z);
  virtual Eigen::MatrixXd jacobianPose(const MobileRobot2dModel * robot_model, const FeatureMap2dModel * map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x);
  virtual Eigen::MatrixXd jacobianFeature(const MobileRobot2dModel * robot_model, const FeatureMap2dModel * map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
};

#endif /* FEATURE_MEASUREMENT_MODEL_H */

