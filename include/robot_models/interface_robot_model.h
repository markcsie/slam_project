#ifndef INTERFACE_ROBOT_MODEL_H
#define INTERFACE_ROBOT_MODEL_H

#include <Eigen/Dense>

#include "../motion_models/interface_motion_model.h"
#include "../measurement_models/interface_measurement_model.h"
#include "../map_models/interface_map_model.h"

class MotionModelInterface;
class MeasurementModelInterface;

class RobotModelInterface
{
public:
  virtual ~RobotModelInterface();
  
  virtual Eigen::VectorXd samplePose(const Eigen::VectorXd &x, const Eigen::VectorXd &u);
  virtual Eigen::VectorXd predictPose(const Eigen::VectorXd &x, const Eigen::VectorXd &u);
  virtual Eigen::VectorXd predictMeasurement(const MapModelInterface *map_2d_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
  virtual Eigen::VectorXd inverseMeasurement(const MapModelInterface *map_2d_model, const Eigen::VectorXd &x, const Eigen::VectorXd &z);
  virtual Eigen::MatrixXd jacobianPose(const MapModelInterface *map_2d_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
  virtual Eigen::MatrixXd jacobianFeature(const MapModelInterface *map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
  
  const std::string &getType() const
  {
    return type_;
  };

  const size_t &getDim() const
  {
    return dim_;
  };

  const Eigen::VectorXd &getInitX() const
  {
    return init_x_;
  };
    
  const Eigen::MatrixXd &getQt() const;

protected:
  RobotModelInterface(); // abstract base class

  
  std::string type_;
  size_t dim_;
  Eigen::VectorXd init_x_;
  
  MotionModelInterface *motion_model_;
  MeasurementModelInterface *measurement_model_;
};

#endif /* INTERFACE_ROBOT_MODEL_H */

