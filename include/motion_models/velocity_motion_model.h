#ifndef VELOCITY_MOTION_MODEL_H
#define VELOCITY_MOTION_MODEL_H

#include "interface_motion_model.h"
#include "../robot_models/mobile_robot_2dmodel.h"

class VelocityMotionModel : public MotionModelInterface
{
public:
  VelocityMotionModel();
  VelocityMotionModel(const VelocityMotionModel& other);
  virtual ~VelocityMotionModel();

  virtual Eigen::VectorXd samplePose(const RobotModelInterface * robot_model, const Eigen::VectorXd &x, const Eigen::VectorXd &u);
  virtual Eigen::VectorXd predictPose(const RobotModelInterface * robot_model, const Eigen::VectorXd &x, const Eigen::VectorXd &u);

private:
  virtual Eigen::VectorXd samplePose(const MobileRobot2dModel * robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u);
  virtual Eigen::VectorXd predictPose(const MobileRobot2dModel * robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u);
};

#endif /* VELOCITY_MOTION_MODEL_H */

