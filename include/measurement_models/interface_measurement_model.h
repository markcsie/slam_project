#ifndef INTERFACE_MEASUREMENT_MODEL_H
#define INTERFACE_MEASUREMENT_MODEL_H

#include <Eigen/Dense>

#include "../robot_models/interface_robot_model.h"
#include "../map_models/interface_map_model.h"

class RobotModelInterface;
class MapModelInterface;

class MeasurementModelInterface
{
public:
  virtual Eigen::VectorXd predictMeasurement(const RobotModelInterface *robot_model, const MapModelInterface *map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x) = 0;
  virtual Eigen::VectorXd inverseMeasurement(const RobotModelInterface *robot_model, const MapModelInterface *map_model, const Eigen::VectorXd &x, const Eigen::VectorXd &z) = 0;
  virtual Eigen::MatrixXd jacobianPose(const RobotModelInterface *robot_model, const MapModelInterface *map_model, const Eigen::VectorXd& mean, const Eigen::VectorXd& x) = 0;
  virtual Eigen::MatrixXd jacobianFeature(const RobotModelInterface *robot_model, const MapModelInterface *map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x) = 0;

  virtual const std::string &getType() const
  {
    return type_;
  };

  virtual const size_t &getDim() const
  {
    return dim_;
  };
  
  virtual const Eigen::MatrixXd &getQt() const
  {
    return Q_t_;
  };
  
protected:
  std::string type_;
  size_t dim_;
  Eigen::MatrixXd Q_t_; // measurement noise
};

#endif /* INTERFACE_MEASUREMENT_MODEL_H */

