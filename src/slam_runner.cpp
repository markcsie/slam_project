#include <ros/ros.h>

#include "fast_slam.h"
#include "fast_slam2.h"
#include "multi_fast_slam.h"
#include "dec_multi_fast_slam.h"

#include "motion_models/velocity_motion_model.h"
#include "measurement_models/feature_measurement_model.h"
#include "slam_project/Robot_Odometry.h"
#include "slam_project/requestBarcode.h"
#include "slam_project/Robot_GroundTruth.h"
#include "slam_project/Robot_Odometry_Single.h"
#include "slam_project/Robot_GroundTruth_Multi.h"
#include "std_msgs/Float64MultiArray.h"
#include "../include/data_reader.h"

#include "../include/utils/eigenmvn.h"
#include <ros/package.h>
std::vector<std::vector<groundtruth> > multirobot_groundtruth;

class SlamRunner
{
public:
  SlamRunner(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const std::vector<RobotModelInterface> &robots, MapModelInterface &map);
  SlamRunner(const SlamRunner& other);
  virtual ~SlamRunner();

  ros::Publisher dataPublisher;
  void frameCallback(const slam_project::Robot_Odometry_Single &msg);
  void multiFrameCallback(const slam_project::Robot_Odometry &msg); //multi-robot
  void readGroundtruth(const int index, const string path);
private:
  FastSlam fast_slam_; // TODO: polymorphism for SLAM algorithm?
  FastSlam2 fast_slam2_; // TODO: polymorphism for SLAM algorithm?
  MultiFastSlam multi_fast_slam_; // TODO: polymorphism for SLAM algorithm?
  DecMultiFastSlam dec_multi_fast_slam_; // TODO: polymorphism for SLAM algorithm?

  std::vector<std::shared_ptr<const RobotModelInterface>> robots_;

  size_t frame_count_;
  double last_time_;

  slam_project::Robot_GroundTruth singlePostProcess();
  slam_project::Robot_GroundTruth_Multi multiPostProcess(const int &frame_id);

};

SlamRunner::SlamRunner(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const std::vector<RobotModelInterface> &robots, MapModelInterface &map)
:
fast_slam_(num_particles, initial_x, initial_cov, initial_w, robots[0], map),
fast_slam2_(num_particles, initial_x, initial_cov, initial_w, robots[0], map),
multi_fast_slam_(num_particles, initial_x, initial_cov, initial_w, robots, map),
dec_multi_fast_slam_(num_particles, initial_x[0], initial_cov, initial_w, robots, map),
frame_count_(0),
last_time_(0.0)
{
  for (const RobotModelInterface & r : robots)
  {
    robots_.push_back(std::shared_ptr<const RobotModelInterface>(&r));
  }
}

SlamRunner::SlamRunner(const SlamRunner& other)
: fast_slam_(other.fast_slam_), fast_slam2_(other.fast_slam2_), multi_fast_slam_(other.multi_fast_slam_), dec_multi_fast_slam_(other.dec_multi_fast_slam_), frame_count_(other.frame_count_)
{
}

SlamRunner::~SlamRunner()
{
}

void SlamRunner::frameCallback(const slam_project::Robot_Odometry_Single &msg)
{
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
  //  std::cout << "ggg " << u.transpose() << std::endl;
  //  std::cout<<"z rows: "<<z.rows()<<std::endl;
  //  for (int i=0; i<z.rows(); i++){
  //    std::cout<<z(i,0)<<" "<<z(i,1)<<" "<<z(i,2)<<std::endl;
  //  }

  assert(std::abs(msg.time - last_time_ - 0.02) < 0.00001 || frame_count_ == 0);
  std::cout << "ggg msg.time " << msg.time << " frame " << frame_count_ << " u " << u.transpose() << std::endl;
  last_time_ = msg.time;
  frame_count_++;

  //  fast_slam_.process(u, z);
  //  const std::vector<Particle> particles = fast_slam_.getParticles();
  //  const size_t num_particles = fast_slam_.getNumParticles();

  fast_slam2_.process(u, z);
  slam_project::Robot_GroundTruth path_msg = singlePostProcess();
  

  dataPublisher.publish(path_msg);
}



//read groundtruth data

void
SlamRunner::readGroundtruth(const int index, const string path_groundtruth)
{
  ifstream file3(path_groundtruth);
  ifstream file3_2(path_groundtruth);
  int line_count3 = 0;
  string line3;
  double g_time, g_x, g_y, g_orientation;
  groundtruth g_cur;
  while (getline(file3, line3))
    ++line_count3;
  for (int i = 0; i < line_count3; i++)
  {
    file3_2 >> g_time >> g_x >> g_y>>g_orientation;
    g_cur.id = i;
    g_cur.time = g_time;
    g_cur.x = g_x;
    g_cur.y = g_y;
    g_cur.orientation = g_orientation;
    multirobot_groundtruth[index].push_back(g_cur);
  }
  file3.close();
  file3_2.close();
}

slam_project::Robot_GroundTruth SlamRunner::singlePostProcess()
{
  const std::vector<Particle> particles = fast_slam2_.getParticles();
  const size_t num_particles = fast_slam2_.getNumParticles();

  Eigen::VectorXd average_x(3);
  average_x << 0, 0, 0;
  std::unordered_map<int, Eigen::VectorXd> features_average_x;
  std::unordered_map<int, Eigen::MatrixXd> features_average_cov;
  for (const Particle &p : particles)
  {
    //    std::cout << "p.x_ " << p.x_ << std::endl;
    average_x += p.x_;
    for (const auto &it : p.features_)
    {
      if (features_average_x.find(it.first) == features_average_x.end())
      {
        features_average_x[it.first] = it.second.mean_;
      }
      else
      {
        features_average_x[it.first] += it.second.mean_;
      }
    }
  }
  average_x /= num_particles;

  for (const auto &it : features_average_x)
  {
    features_average_x[it.first] /= num_particles;
    Eigen::MatrixXd particles_matrix(num_particles, 2);
    size_t i = 0;
    for (const Particle &p : particles)
    {
      particles_matrix.row(i) = p.features_.find(it.first)->second.mean_.transpose();
      i++;
    }

    if (num_particles > 1)
    {
      Eigen::MatrixXd temp = particles_matrix - Eigen::VectorXd::Ones(particles_matrix.rows(), 1) * features_average_x[it.first].transpose();
      features_average_cov[it.first] = temp.transpose() * temp;
      features_average_cov[it.first] /= (particles_matrix.rows() - 1);
    }
    else
    {
      features_average_cov[it.first] = particles[0].features_.find(it.first)->second.covariance_;
    }

    //    cout << "ggg feature " << it.first << " x " << features_average_x[it.first].transpose() << endl;
    //    cout << "ggg feature " << it.first << " cov " << std::endl << features_average_cov[it.first] << endl;
  }

  //  cout << "ggg particles average posterior " << average_x << endl;

  slam_project::Robot_GroundTruth msg2;
  msg2.x = average_x[0];
  msg2.y = average_x[1];

  msg2.num = features_average_x.size(); //TODO
  msg2.landmark_x.clear();
  msg2.landmark_y.clear();
  msg2.landmark_cov.clear();

  msg2.landmark_x.resize(msg2.num); //TODO
  msg2.landmark_y.resize(msg2.num);
  msg2.landmark_cov.resize(msg2.num);

  int i = 0;
  //  cout << "map size: " << average_features.size() << endl;
  for (auto n = features_average_x.begin(); n != features_average_x.end(); ++n)
  {
    /*2*2 matrix 
      n->second.covariance
      calculate eigen value, eigen vector

      filter nan value
     */
    //  for( const auto& n : p.features_ ) {
    msg2.landmark_x[i] = n->second[0];
    msg2.landmark_y[i] = n->second[1];
    //    cout << "ggg ******* landmark " << n->second.mean_[0] << " " << n->second.mean_[1] << endl;

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
    msg2.landmark_cov[i].data[0] = features_average_cov[n->first](0, 0);
    msg2.landmark_cov[i].data[1] = features_average_cov[n->first](0, 1);
    msg2.landmark_cov[i].data[2] = features_average_cov[n->first](1, 0);
    msg2.landmark_cov[i].data[3] = features_average_cov[n->first](1, 1);
    //    std::cout << "ggg ******* landmark cov " << std::endl;
    //    std::cout << n->second.covariance_ << std::endl;

    i++;
  }
  
  return msg2;
}

slam_project::Robot_GroundTruth_Multi SlamRunner::multiPostProcess(const int &frame_id)
{
  const std::vector<MultiRobotParticle> particles = multi_fast_slam_.getParticles();
  const size_t num_particles = multi_fast_slam_.getNumParticles();

  std::vector<Eigen::VectorXd> average_x(robots_.size(), Eigen::VectorXd::Zero(3));
  std::unordered_map<int, Eigen::VectorXd> features_average_x;
  std::unordered_map<int, Eigen::MatrixXd> features_average_cov;
  for (const MultiRobotParticle &p : particles)
  {
    for (size_t i = 0; i < robots_.size(); i++)
    {
      average_x[i] += p.x_.find(robots_[i]->getId())->second;
    }

    for (const auto &it : p.features_)
    {
      if (features_average_x.find(it.first) == features_average_x.end())
      {
        features_average_x[it.first] = it.second.mean_;
      }
      else
      {
        features_average_x[it.first] += it.second.mean_;
      }
    }
  }

  for (size_t i = 0; i < robots_.size(); i++)
  {
    average_x[i] /= num_particles;
  }


  for (const auto &it : features_average_x)
  {
    features_average_x[it.first] /= num_particles;
    Eigen::MatrixXd particles_matrix(num_particles, 2);
    size_t i = 0;
    for (const MultiRobotParticle &p : particles)
    {
      particles_matrix.row(i) = p.features_.find(it.first)->second.mean_.transpose();
      i++;
    }

    if (num_particles > 1)
    {
      Eigen::MatrixXd temp = particles_matrix - Eigen::VectorXd::Ones(particles_matrix.rows(), 1) * features_average_x[it.first].transpose();
      features_average_cov[it.first] = temp.transpose() * temp;
      features_average_cov[it.first] /= (particles_matrix.rows() - 1);
    }
    else
    {
      features_average_cov[it.first] = particles[0].features_.find(it.first)->second.covariance_;
    }

  }
  // TODO: multi robot
  int cur_id = frame_id;
  int robot_num = multirobot_groundtruth.size();
  slam_project::Robot_GroundTruth_Multi msg2;
  msg2.rnum = robot_num;
  msg2.x.resize(robot_num);
  msg2.y.resize(robot_num);
  msg2.rx.resize(robot_num);
  msg2.ry.resize(robot_num);
  for (int i = 0; i < robot_num; i++)
  {
    msg2.x[i] = average_x[i][0];
    msg2.y[i] = average_x[i][1];
  }
  for (int i = 0; i < robot_num; i++)
  {
    msg2.rx[i] = multirobot_groundtruth[i][cur_id].x;
    msg2.ry[i] = multirobot_groundtruth[i][cur_id].y;
  }

  msg2.num = features_average_x.size(); //TODO
  msg2.landmark_x.clear();
  msg2.landmark_y.clear();
  msg2.landmark_cov.clear();

  msg2.landmark_x.resize(msg2.num); //TODO
  msg2.landmark_y.resize(msg2.num);
  msg2.landmark_cov.resize(msg2.num);


  int i = 0;
  //  cout << "map size: " << average_features.size() << endl;
  for (auto n = features_average_x.begin(); n != features_average_x.end(); ++n)
  {
    //2*2 matrix 
    //  n->second.covariance
    //  calculate eigen value, eigen vector

    //  filter nan value

    //  for( const auto& n : p.features_ ) {
    msg2.landmark_x[i] = n->second[0];
    msg2.landmark_y[i] = n->second[1];
    //    cout << "ggg ******* landmark " << n->second.mean_[0] << " " << n->second.mean_[1] << endl;

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
    msg2.landmark_cov[i].data[0] = features_average_cov[n->first](0, 0);
    msg2.landmark_cov[i].data[1] = features_average_cov[n->first](0, 1);
    msg2.landmark_cov[i].data[2] = features_average_cov[n->first](1, 0);
    msg2.landmark_cov[i].data[3] = features_average_cov[n->first](1, 1);
    //    std::cout << "ggg ******* landmark cov " << std::endl;
    //    std::cout << n->second.covariance_ << std::endl;

    i++;
  }

  return msg2;
}

void SlamRunner::multiFrameCallback(const slam_project::Robot_Odometry &msg)
{
  // TODO: multi-robot data
  //  cout << msg.forward_velocity[0] << endl;
  std::vector<Eigen::VectorXd> multi_u;
  std::vector<Eigen::MatrixXd> multi_z;
  for (int i = 0; i < msg.robot_num; i++)
  {
    Eigen::VectorXd u(2);
    u[0] = msg.forward_velocity[i];
    u[1] = msg.angular_velocity[i];
    multi_u.push_back(u);

    int len = msg.num[i];
    Eigen::MatrixXd z(len, 3);
    for (int j = 0; j < len; j++)
    {
      z(j, 0) = msg.measure[i].subject[j];
      z(j, 1) = msg.measure[i].range[j];
      z(j, 2) = msg.measure[i].bearing[j];
    }
    multi_z.push_back(z);
  }


  //  std::cout << "ggg msg.time.size() " << msg.time.size() << std::endl;
  //  std::cout << "ggg msg.time[0] " << msg.time[0] << std::endl;
  //  std::cout << "ggg frame_count_ " << frame_count_ << std::endl;
  assert(std::abs(msg.time[0] - last_time_ - 0.02) < 0.00001 || frame_count_ == 0);
  last_time_ = msg.time[0];
  frame_count_++;

  multi_fast_slam_.process(multi_u, multi_z);
  slam_project::Robot_GroundTruth_Multi path_msg = multiPostProcess(msg.id);
  dataPublisher.publish(path_msg);

  //  dec_multi_fast_slam_.process(multi_u, multi_z);
  //  const std::vector<MultiRobotParticle> particles = dec_multi_fast_slam_.getParticles();
  //  const size_t num_particles = dec_multi_fast_slam_.getNumParticles();
}

int main(int argc, char **argv)
{
  // =============testing  code
  // test sampleGaussian
  //  std::vector<double> gaussian_samples;
  //  for (size_t i = 0; i < 10000; i++)
  //  {
  //    gaussian_samples.push_back(Utils::sampleGaussian(10, 5));
  ////    std::cout << gaussian_samples[i] << std::endl;
  //  }
  //  double gaussian_sum = std::accumulate(gaussian_samples.begin(), gaussian_samples.end(), 0.0);
  //  double gaussian_mean = gaussian_sum / gaussian_samples.size();
  //  std::cout << "gaussian_mean " << gaussian_mean << std::endl;
  //
  //  double gaussian_sq_sum = std::inner_product(gaussian_samples.begin(), gaussian_samples.end(), gaussian_samples.begin(), 0.0);
  //  double gaussian_var = gaussian_sq_sum / gaussian_samples.size() - gaussian_mean * gaussian_mean;
  //  double gaussian_stdev = std::sqrt(gaussian_var);
  //  std::cout << "gaussian_var " << gaussian_var << std::endl;
  //  std::cout << "gaussian_stdev " << gaussian_stdev << std::endl;
  //  
  //  // test sampleUniform
  //  std::vector<double> uniform_samples;
  //  for (size_t i = 0; i < 10000; i++)
  //  {
  //    uniform_samples.push_back(Utils::sampleUniform(5, 15));
  ////    std::cout << uniform_samples[i] << std::endl;
  //  }
  //  double uniform_sum = std::accumulate(uniform_samples.begin(), uniform_samples.end(), 0.0);
  //  double uniform_mean = uniform_sum / uniform_samples.size();
  //  std::cout << "uniform_mean " << uniform_mean << std::endl;
  //  std::cout << "uniform_max " << *std::max_element(uniform_samples.begin(), uniform_samples.end()) << std::endl;
  //  std::cout << "uniform_min " << *std::min_element(uniform_samples.begin(), uniform_samples.end()) << std::endl;
  //  
  // test Eigen::EigenMultivariateNormal<double> norm(mean, covariance);
  //  Eigen::MatrixXd mvn_samples(20000, 3);
  //  Eigen::VectorXd mvn_mean(3);
  //  mvn_mean << 1, 2, 3;
  //  Eigen::MatrixXd mvn_covariance(3, 3);
  //  mvn_covariance << 1, 2, 3,
  //                    2, 5, 6,
  //                    3, 6, 9;
  //  
  //  for (size_t i = 0; i < mvn_samples.rows(); i++)
  //  {
  //    Eigen::EigenMultivariateNormal<double> norm(mvn_mean, mvn_covariance);
  //    mvn_samples.row(i) = norm.samples(1).transpose();
  //  }
  //  Eigen::VectorXd mvn_sum(3);
  //  mvn_sum << 0, 0, 0;
  //  for (size_t i = 0; i < mvn_samples.rows(); i++) {
  //    mvn_sum += mvn_samples.row(i).transpose();
  //  }
  //  Eigen::VectorXd sample_mvn_mean = mvn_sum / mvn_samples.rows();
  //  std::cout << "sample_mvn_mean " << sample_mvn_mean.transpose() << std::endl;
  //  Eigen::MatrixXd sample_mvn_cov = (mvn_samples - Eigen::VectorXd::Ones(mvn_samples.rows(), 1) * sample_mvn_mean.transpose()).transpose() * (mvn_samples - Eigen::VectorXd::Ones(mvn_samples.rows(), 1) * sample_mvn_mean.transpose());
  //  sample_mvn_cov = sample_mvn_cov / (mvn_samples.rows() - 1);
  //  std::cout << "sample_mvn_cov " << std::endl;
  //  std::cout << sample_mvn_cov << std::endl;

  //  std::cout << "Utils::sampleGaussian(0, 0) " << Utils::sampleGaussian(0, 0) << std::endl;
  // =============

  //  std::cout.precision(20);

  //read groundtruth data



  ros::init(argc, argv, "slam_runner");
  ros::NodeHandle node("~");

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

  std::vector<RobotModelInterface> robots;
  robots.push_back(MobileRobot2dModel(5, velocity_model, feature_model)); // robot 1 // TODO: different models?
  robots.push_back(MobileRobot2dModel(14, velocity_model, feature_model)); // robot 2 // TODO: different models?

  std::vector<Eigen::VectorXd> initial_x(robots.size(), Eigen::VectorXd::Zero(3)); // TODO: parameter or random, particles_[i].x_ = robot.getRandomX(map_);
  initial_x[0] << 1.916028, -2.676211, 0.390500;
  initial_x[1] << 0.611662, -3.520002, 0.319200;

  Eigen::MatrixXd initial_cov(3, 3); // TODO: parameter or random?
  initial_cov << 0.000001, 0, 0,
          0, 0.000001, 0,
          0, 0, 0.000001;
  SlamRunner slam_runner(num_particles, initial_x, initial_cov, initial_w, robots, map); // TODO: dimension depends on the incoming data

  //groundtruth
  int robot_num = 2;
  multirobot_groundtruth.resize(robot_num);
  string path = ros::package::getPath("slam_project");
  for (int i = 1; i <= robot_num; i++)
  {
    stringstream ss;
    ss << i;
    string str = ss.str();
    string path_groundtruth = path + "/data/newRobot" + str + "_Groundtruth.txt";
    slam_runner.readGroundtruth(i - 1, path_groundtruth);
  }


  //ros::Subscriber frame_sub = node.subscribe("/publishMsg2", 100000, &SlamRunner::frameCallback, &slam_runner);
  ros::Subscriber frame_sub = node.subscribe("/publishMsg2", 100000, &SlamRunner::multiFrameCallback, &slam_runner);
  slam_runner.dataPublisher = node.advertise<slam_project::Robot_GroundTruth_Multi>("/publishMsg4", 100000);
  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce(); // check for incoming messages
  }

  return EXIT_SUCCESS;
}