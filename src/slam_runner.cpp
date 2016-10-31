#include <ros/ros.h>

#include "fast_slam2.h"

#include "motion_models/velocity_motion_model.h"
#include "measurement_models/feature_measurement_model.h"
#include "slam_project/Robot_Odometry.h"
#include "slam_project/requestBarcode.h"

#include "std_msgs/String.h" // temporary

class SlamRunner
{
public:
  SlamRunner(const size_t &num_particles, const double &initial_w, RobotModelInterface &robot, MapModelInterface &map);
  SlamRunner(const SlamRunner& other);
  virtual ~SlamRunner();

  void frameCallback(const slam_project::Robot_Odometry &msg);

private:
  FastSlam2 fast_slam2_; // TODO: polymorphism for SLAM algorithm?
};

SlamRunner::SlamRunner(const size_t &num_particles, const double &initial_w, RobotModelInterface &robot, MapModelInterface &map)
: fast_slam2_(num_particles, initial_w, robot, map)
{
  // TODO: This is just testing
  // Eigen::VectorXd u;
  // Eigen::MatrixXd z;
  // fast_slam2_.process(u, z);
}

SlamRunner::SlamRunner(const SlamRunner& other)
: fast_slam2_(other.fast_slam2_)
{
}

SlamRunner::~SlamRunner()
{
}

void SlamRunner::frameCallback(const slam_project::Robot_Odometry &msg){
// TODO: process the message and run the slam algorithm
//  fast_slam2.process()
  Eigen::VectorXd u(2);
  u[0] = msg.forward_velocity;
  u[1] = msg.angular_velocity;

  int len = msg.num;
  Eigen::MatrixXd z(len, 3);
  for (int i=0; i<len; i++){
    z(i,0) = msg.subject[i];
    z(i,1) = msg.range[i];
    z(i,2) = msg.bearing[i];
  }
//  std::cout<<u[0]<<" "<<u[1]<<std::endl;
//  std::cout<<"z rows: "<<z.rows()<<std::endl;
//  for (int i=0; i<z.rows(); i++){
//    std::cout<<z(i,0)<<" "<<z(i,1)<<" "<<z(i,2)<<std::endl;
//  }  
  fast_slam2_.process(u, z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam_runner");
  ros::NodeHandle node("~");

  ros::Rate rate(1.0);
  ROS_INFO("start spinning");

  int num_particles;
  node.param<int>("slam/num_particles", num_particles, 100);
  std::cout << "num_particles " << num_particles << std::endl;
  ROS_ASSERT(num_particles > 0);

  double initial_w;
  node.param<double>("slam/initial_w", initial_w, 1.0);
  std::cout << "initial_w " << initial_w << std::endl;
  ROS_ASSERT(initial_w > 0);

  // TODO: from file
  Eigen::VectorXd map_size(2);
  map_size << 8, 15;
  Eigen::VectorXd map_corner(2);
  map_corner << -2, -6;
  FeatureMap2dModel map(map_size, map_corner);
  
  std::vector<double> measurement_noise;
  node.param("slam/measurement_noise", measurement_noise, {1.0, 1.0});
  Eigen::MatrixXd Q_t(2, 2);
  Q_t << measurement_noise[0], 0.0,
         0.0, measurement_noise[1];
  FeatureMeasurementModel feature_model(Q_t);
  
  std::vector<double> motion_noise;
  node.param("slam/motion_noise", motion_noise, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  double delta_t = 0.02; // TODO: from data_reader
  VelocityMotionModel velocity_model(motion_noise, delta_t);

  MobileRobot2dModel robot(velocity_model, feature_model);
  
  SlamRunner slam_runner(num_particles, initial_w, robot, map); // TODO: dimension depends on the incoming data

  ros::Subscriber frame_sub = node.subscribe("/publishMsg2", 100, &SlamRunner::frameCallback, &slam_runner);


  while (ros::ok())
  {
    ros::spinOnce(); // check for incoming messages
    rate.sleep();
  }

  return EXIT_SUCCESS;
}