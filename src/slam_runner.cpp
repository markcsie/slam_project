#include <ros/ros.h>

#include "fast_slam2.h"

#include "motion_models/velocity_motion_model.h"
#include "measurement_models/feature_measurement_model.h"

#include "std_msgs/String.h" // temporary

class SlamRunner
{
public:
  SlamRunner(const size_t &num_particles, const double &initial_w, RobotModelInterface &robot, MapModelInterface &map);
  SlamRunner(const SlamRunner& other);
  virtual ~SlamRunner();

  void frameCallback(const std_msgs::String &msg);

private:
  FastSlam2 fast_slam2_; // TODO: polymorphism for SLAM algorithm?
};

SlamRunner::SlamRunner(const size_t &num_particles, const double &initial_w, RobotModelInterface &robot, MapModelInterface &map)
: fast_slam2_(num_particles, initial_w, robot, map)
{
  // TODO: This is just testing
  Eigen::VectorXd u;
  Eigen::MatrixXd z;
  fast_slam2_.process(u, z);
}

SlamRunner::SlamRunner(const SlamRunner& other)
: fast_slam2_(other.fast_slam2_)
{
}

SlamRunner::~SlamRunner()
{
}

void SlamRunner::frameCallback(const std_msgs::String &msg)
{
  // TODO: process the message and run the slam algorithm
  //  fast_slam2.process()
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_reader");
  ros::NodeHandle node("~");

  ros::Rate rate(1.0);
  ROS_INFO("start spinning");

  int num_particles;
  node.param<int>("num_particles", num_particles, 100);
  std::cout << "num_particles " << num_particles << std::endl;
  ROS_ASSERT(num_particles > 0);

  double initial_w;
  node.param<double>("initial_w", initial_w, 1.0);
  std::cout << "initial_w " << initial_w << std::endl;
  ROS_ASSERT(initial_w > 0);

  FeatureMap2dModel map;
  Eigen::MatrixXd Q_t(2, 2); // TODO:
  Q_t << 1.0, 1.0,
         1.0, 1.0;
  FeatureMeasurementModel feature_model(Q_t);
  std::vector<double> alphas = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // TODO:
  double delta_t = 1.0; // TODO: from data_reader
  VelocityMotionModel velocity_model(alphas, delta_t);
  MobileRobot2dModel robot(velocity_model, feature_model);
  
  SlamRunner slam_runner(num_particles, initial_w, robot, map); // TODO: dimension depends on the incoming data

  ros::Subscriber frame_sub = node.subscribe("TODO", 1, &SlamRunner::frameCallback, &slam_runner);

  while (ros::ok())
  {
    ros::spinOnce(); // check for incoming messages
    rate.sleep();
  }

  return EXIT_SUCCESS;
}