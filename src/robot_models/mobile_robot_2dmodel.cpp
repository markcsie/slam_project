#include "robot_models/mobile_robot_2dmodel.h"

const std::string MobileRobot2dModel::TYPE = "2D Mobile Robot Model";

MobileRobot2dModel::MobileRobot2dModel(MotionModelInterface &motion_model, MeasurementModelInterface &measurement_model)
{
  type_ = MobileRobot2dModel::TYPE;
  dim_ = 3;
  motion_model_ = &motion_model;
  measurement_model_ = &measurement_model;
}

MobileRobot2dModel::MobileRobot2dModel(const MobileRobot2dModel& other)
{

}

MobileRobot2dModel::~MobileRobot2dModel()
{

}