#include "robot_models/mobile_robot_2dmodel.h"

const std::string MobileRobot2dModel::TYPE = "2D Mobile Robot Model";

MobileRobot2dModel::MobileRobot2dModel(const int &id, const MotionModelInterface &motion_model, const MeasurementModelInterface &measurement_model)
{
  type_ = MobileRobot2dModel::TYPE;
  id_ = id;
  dim_ = 3;
  motion_model_ = std::shared_ptr<const MotionModelInterface>(&motion_model);
  measurement_model_ = std::shared_ptr<const MeasurementModelInterface>(&measurement_model);
}

MobileRobot2dModel::MobileRobot2dModel(const MobileRobot2dModel& other)
{

}

MobileRobot2dModel::~MobileRobot2dModel()
{
}