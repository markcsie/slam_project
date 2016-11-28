#ifndef FAST_SLAM2_H
#define FAST_SLAM2_H

#include <unordered_map>
#include <memory>

#include <Eigen/Dense>

#include "particle.h"
#include "robot_models/mobile_robot_2dmodel.h"
#include "map_models/interface_map_model.h"

class FastSlam2
{
public:
  FastSlam2(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const std::shared_ptr<const RobotModelInterface> &robot, const MapModelInterface &map);
  FastSlam2(const FastSlam2& other);
  virtual ~FastSlam2();

  size_t getNumParticles();
  std::vector<Particle> getParticles();
  Particle getParticle(const size_t &i);
  void process(const Eigen::VectorXd &u, const Eigen::MatrixXd &features);
private:
  std::shared_ptr<const RobotModelInterface> robot_;
  std::shared_ptr<const MapModelInterface> map_;
  int robot_id_;

  double initial_w_;
  std::vector<Particle> particles_;

  double updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature);

  Eigen::VectorXd sampleMultivariateGaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance) const;
};

#endif /* FAST_SLAM2_H */

