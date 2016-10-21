#ifndef VELOCITY_MOTION_MODEL_H
#define VELOCITY_MOTION_MODEL_H

#include "interface_motion_model.h"
#include "../robot_models/mobile_robot_2dmodel.h"

class VelocityMotionModel : public MotionModelInterface
{
public:
  VelocityMotionModel(const std::vector<double> &alphas, const double &delta_t);
  VelocityMotionModel(const VelocityMotionModel& other);
  virtual ~VelocityMotionModel();

  virtual Eigen::VectorXd samplePose(const RobotModelInterface * robot_model, const Eigen::VectorXd &x, const Eigen::VectorXd &u);
  virtual Eigen::VectorXd predictPose(const RobotModelInterface * robot_model, const Eigen::VectorXd &x, const Eigen::VectorXd &u);

private:
  virtual Eigen::VectorXd samplePose(const MobileRobot2dModel * robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u);
  virtual Eigen::VectorXd predictPose(const MobileRobot2dModel * robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u);
  
  double delta_t_;
  std::vector<double> alphas_; // parameters fo motion noise 
};

#endif /* VELOCITY_MOTION_MODEL_H */

