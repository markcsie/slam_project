#ifndef FAST_SLAM2_H
#define FAST_SLAM2_H

#include <unordered_map>
#include <memory>

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
  FastSlam2(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const double &initial_w, const RobotModelInterface &robot, const MapModelInterface &map);
  FastSlam2(const FastSlam2& other);
  virtual ~FastSlam2();

  void process(const Eigen::VectorXd &u, const Eigen::MatrixXd &z);
private:
  std::shared_ptr<const RobotModelInterface> robot_;
  std::shared_ptr<const MapModelInterface> map_;

  double initial_w_;
  std::vector<Particle> particles_;

  void updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::MatrixXd &features);

  Eigen::VectorXd sampleMultivariateGaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance) const;

  Eigen::MatrixXd calculateRt(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
  Eigen::VectorXd samplePose(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
  Eigen::VectorXd predictPose(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
  Eigen::VectorXd predictMeasurement(const Eigen::VectorXd &mean, const Eigen::VectorXd &x)const;
  Eigen::VectorXd inverseMeasurement(const Eigen::VectorXd &x, const Eigen::VectorXd &z) const;
  Eigen::MatrixXd jacobianPose(const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;
  Eigen::MatrixXd jacobianFeature(const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;
  
  bool dead_reckoning_;
};

#endif /* FAST_SLAM2_H */

