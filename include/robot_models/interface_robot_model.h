#ifndef INTERFACE_ROBOT_MODEL_H
#define INTERFACE_ROBOT_MODEL_H

#include <memory>
#include <Eigen/Dense>

#include "../utils/random_utils.h"

#include "../motion_models/interface_motion_model.h"
#include "../measurement_models/interface_measurement_model.h"
#include "../map_models/interface_map_model.h"

class MotionModelInterface;
class MeasurementModelInterface;

class RobotModelInterface : public std::enable_shared_from_this<RobotModelInterface>
{
public:
  virtual ~RobotModelInterface();
  
  virtual Eigen::MatrixXd calculateRt(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
  virtual Eigen::VectorXd samplePose(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
  virtual Eigen::VectorXd predictPose(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
  virtual Eigen::VectorXd predictMeasurement(const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;
  virtual Eigen::VectorXd inverseMeasurement(const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd &x, const Eigen::VectorXd &z) const;
  virtual Eigen::MatrixXd jacobianPose(const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;
  virtual Eigen::MatrixXd jacobianFeature(const std::shared_ptr<const MapModelInterface> &map_model, const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;
  
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
  
  Eigen::VectorXd getRandomX(const std::shared_ptr<const MapModelInterface> &map) const 
  {
    Eigen::VectorXd p = map->getRandomPosition();
    assert(dim_ >= map->getDim());
    p.conservativeResize(dim_, Eigen::NoChange);
    
    // orientation
    for (size_t i = map->getDim(); i < p.rows(); i++) {
      p[i] = Utils::sampleUniform(-M_PI, +M_PI);
    }

    return p;
  };
  
protected:
  RobotModelInterface(); // abstract base class

  std::string type_;
  size_t dim_;
  Eigen::VectorXd init_x_;
  
  std::shared_ptr<const MotionModelInterface> motion_model_;
  std::shared_ptr<const MeasurementModelInterface> measurement_model_;
};

#endif /* INTERFACE_ROBOT_MODEL_H */

