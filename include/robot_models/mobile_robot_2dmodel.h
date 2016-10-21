#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "interface_robot_model.h"

class MobileRobot2dModel : public RobotModelInterface
{
public:
  MobileRobot2dModel(MotionModelInterface &motion_model, MeasurementModelInterface &measurement_model);
  MobileRobot2dModel(const MobileRobot2dModel& other);
  virtual ~MobileRobot2dModel();
  
  static const std::string TYPE;
};

#endif /* ROBOT_MODEL_H */

