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

  virtual Eigen::MatrixXd calculateRt(const std::shared_ptr<const RobotModelInterface> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;
  virtual Eigen::VectorXd samplePose(const std::shared_ptr<const RobotModelInterface> &robot_model, const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
  virtual Eigen::VectorXd predictPose(const std::shared_ptr<const RobotModelInterface> &robot_model, const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
    
private:
  virtual Eigen::MatrixXd calculateRt(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;
  virtual Eigen::VectorXd samplePose(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;
  virtual Eigen::VectorXd predictPose(const std::shared_ptr<const MobileRobot2dModel> &robot_model, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;
  
  double delta_t_;
  std::vector<double> alphas_; // parameters fo motion noise 
};

#endif /* VELOCITY_MOTION_MODEL_H */

