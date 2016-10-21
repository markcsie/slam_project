#ifndef FAST_SLAM2_H
#define FAST_SLAM2_H

#include <vector>
#include <unordered_map>
#include <random>

#include <Eigen/Dense>

#include "robot_models/mobile_robot_2dmodel.h"
#include "map_models/interface_map_model.h"

struct Gaussian
{
  Eigen::VectorXd mean_;
  Eigen::MatrixXd covariance_;
};

struct Particle
{
  Eigen::VectorXd x_;
  std::unordered_map<int, Gaussian> features_; // key = subject # value Gaussian(mean, covariance)
  double w_; // importance weight
};

class FastSlam2
{
public:
  FastSlam2(const size_t &num_particles, const double &initial_w, RobotModelInterface &robot, MapModelInterface &map);
  FastSlam2(const FastSlam2& other);
  virtual ~FastSlam2();

  void process(const Eigen::VectorXd &u, const Eigen::MatrixXd &z);
private:
  RobotModelInterface *robot_;
  MapModelInterface *map_;

  std::default_random_engine generator;
  double initial_w_;
  std::vector<Particle> particles_;

  void updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::MatrixXd &z);

  Eigen::VectorXd sampleMultivariateGaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance);

  Eigen::VectorXd samplePose(const Eigen::VectorXd &x, const Eigen::VectorXd &u);
  Eigen::VectorXd predictPose(const Eigen::VectorXd &x, const Eigen::VectorXd &u);
  Eigen::VectorXd predictMeasurement(const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
  Eigen::VectorXd inverseMeasurement(const Eigen::VectorXd &x, const Eigen::VectorXd &z);
  Eigen::MatrixXd jacobianPose(const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
  Eigen::MatrixXd jacobianFeature(const Eigen::VectorXd &mean, const Eigen::VectorXd &x);
};

#endif /* FAST_SLAM2_H */

