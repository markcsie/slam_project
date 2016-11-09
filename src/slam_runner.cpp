#include <ros/ros.h>

#include "fast_slam2.h"

#include "motion_models/velocity_motion_model.h"
#include "measurement_models/feature_measurement_model.h"
#include "slam_project/Robot_Odometry.h"
#include "slam_project/requestBarcode.h"
#include "slam_project/Robot_GroundTruth.h"
#include "std_msgs/Float64MultiArray.h"
#include "../include/data_reader.h"

//global variable
vector<groundtruth> robot_groundtruth;
groundtruth g_cur;

class SlamRunner
{
public:
  SlamRunner(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const double &initial_w, RobotModelInterface &robot, MapModelInterface &map);
  SlamRunner(const SlamRunner& other);
  virtual ~SlamRunner();

  Particle getParticle(const size_t &i);
  ros::Publisher dataPublisher;
  slam_project::Robot_GroundTruth msg2;
  void frameCallback(const slam_project::Robot_Odometry &msg);

private:
  FastSlam2 fast_slam2_; // TODO: polymorphism for SLAM algorithm?
  size_t frame_count_;
};

SlamRunner::SlamRunner(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const double &initial_w, RobotModelInterface &robot, MapModelInterface &map)
: fast_slam2_(num_particles, initial_x, initial_w, robot, map), frame_count_(0)
{
}

SlamRunner::SlamRunner(const SlamRunner& other)
: fast_slam2_(other.fast_slam2_), frame_count_(other.frame_count_)
{
}

SlamRunner::~SlamRunner()
{
}

Particle SlamRunner::getParticle(const size_t &i)
{
  return fast_slam2_.getParticle(i);
}

void SlamRunner::frameCallback(const slam_project::Robot_Odometry &msg)
{
  frame_count_++;
  // TODO: process the message and run the slam algorithm
  //  fast_slam2.process()
  Eigen::VectorXd u(2);
  u[0] = msg.forward_velocity;
  u[1] = msg.angular_velocity;

  int len = msg.num;
  Eigen::MatrixXd z(len, 3);
  for (int i = 0; i < len; i++)
  {
    z(i, 0) = msg.subject[i];
    z(i, 1) = msg.range[i];
    z(i, 2) = msg.bearing[i];
  }
  //  std::cout<<u[0]<<" "<<u[1]<<std::endl;
  //  std::cout<<"z rows: "<<z.rows()<<std::endl;
  //  for (int i=0; i<z.rows(); i++){
  //    std::cout<<z(i,0)<<" "<<z(i,1)<<" "<<z(i,2)<<std::endl;
  //  }  
  std::cout << "ggg frame " << frame_count_ << std::endl;
  fast_slam2_.process(u, z);

  Particle p = fast_slam2_.getParticle(0);
  // TODO: publish for visualization
  // p.x_
  g_cur.x = p.x_[0];
  g_cur.y = p.x_[1];
  //  cout<<g_cur.x<<" "<<g_cur.y<<endl;
  msg2.x = p.x_[0];
  msg2.y = p.x_[1];

  msg2.num = p.features_.size(); //TODO
  msg2.landmark_x.clear();
  msg2.landmark_y.clear();
  msg2.landmark_cov.clear();
  
  msg2.landmark_x.resize(msg2.num); //TODO
  msg2.landmark_y.resize(msg2.num);
  msg2.landmark_cov.resize(msg2.num);

  int i = 0;
  cout << "map size: " << p.features_.size() << endl;
  for (auto n = p.features_.begin(); n != p.features_.end(); ++n)
  {
    /*2*2 matrix 
      n->second.covariance
      calculate eigen value, eigen vector

      filter nan value
    */
    //  for( const auto& n : p.features_ ) {
    msg2.landmark_x[i] = n->second.mean_[0];
    cout << "*******" << n->second.mean_[0] << endl;
    msg2.landmark_y[i] = n->second.mean_[1];

    msg2.landmark_cov[i].layout.dim.clear();
    msg2.landmark_cov[i].layout.dim.resize(2);
    msg2.landmark_cov[i].layout.dim[0].label = "row";
    msg2.landmark_cov[i].layout.dim[0].size = 2;
    msg2.landmark_cov[i].layout.dim[0].stride = 4;
    msg2.landmark_cov[i].layout.dim[1].label = "col";
    msg2.landmark_cov[i].layout.dim[1].size = 2;
    msg2.landmark_cov[i].layout.dim[1].stride = 2;
    msg2.landmark_cov[i].layout.data_offset = 0;
    msg2.landmark_cov[i].data.clear();
    msg2.landmark_cov[i].data.resize(4);
    msg2.landmark_cov[i].data[0] = n->second.covariance_(0, 0);
    msg2.landmark_cov[i].data[1] = n->second.covariance_(0, 1);
    msg2.landmark_cov[i].data[2] = n->second.covariance_(1, 0);
    msg2.landmark_cov[i].data[3] = n->second.covariance_(1, 1);

    i++;
  }

  dataPublisher.publish(msg2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam_runner");
  ros::NodeHandle node("~");

  ros::Rate rate(100);
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
  node.param("slam/measurement_noise", measurement_noise,{1.0, 1.0});
  Eigen::MatrixXd Q_t(2, 2);
  Q_t << measurement_noise[0], 0.0,
          0.0, measurement_noise[1];
  FeatureMeasurementModel feature_model(Q_t);

  std::vector<double> motion_noise;
  node.param("slam/motion_noise", motion_noise,{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  double delta_t = 0.02; // TODO: from data_reader
  VelocityMotionModel velocity_model(motion_noise, delta_t);

  MobileRobot2dModel robot(velocity_model, feature_model);

  Eigen::VectorXd initial_x(3); // TODO: parameter or random, particles_[i].x_ = robot.getRandomX(map_);
  initial_x << 1.916028, -2.676211, 0.390500;
  SlamRunner slam_runner(num_particles, std::vector<Eigen::VectorXd>(num_particles, initial_x), initial_w, robot, map); // TODO: dimension depends on the incoming data

  ros::Subscriber frame_sub = node.subscribe("/publishMsg2", 100, &SlamRunner::frameCallback, &slam_runner);
  slam_runner.dataPublisher = node.advertise<slam_project::Robot_GroundTruth>("/publishMsg4", 1000);

  int i = 0;
  while (ros::ok())
  {
    ros::spinOnce(); // check for incoming messages
    rate.sleep();
  }

  return EXIT_SUCCESS;
}