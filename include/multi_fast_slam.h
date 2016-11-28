#ifndef MULTI_FAST_SLAM_H
#define MULTI_FAST_SLAM_H

#include <unordered_map>
#include <memory>

#include <Eigen/Dense>

#include "particle.h"
#include "robot_models/mobile_robot_2dmodel.h"
#include "map_models/interface_map_model.h"
#include <sstream>

// Multi-robot SLAM with known initial poses
class MultiFastSlam
{
public:
  MultiFastSlam(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const std::vector<RobotModelInterface> &robots, const MapModelInterface &map);
  MultiFastSlam(const MultiFastSlam& other);
  virtual ~MultiFastSlam();

  size_t getNumParticles();
  std::vector<Particle> getParticles();
  Particle getParticle(const size_t &i);
  void process(const std::vector<Eigen::VectorXd> &u, const std::vector<Eigen::MatrixXd> &features);
private:
  std::vector<std::shared_ptr<const RobotModelInterface>> robots_;
  std::shared_ptr<const MapModelInterface> map_;

  double initial_w_;
  std::vector<Particle> particles_;

  std::vector<double> updateRobot(const std::shared_ptr<const RobotModelInterface> &robot, const Eigen::VectorXd &u, const Eigen::MatrixXd &features);
  double updateParticle(const std::shared_ptr<const RobotModelInterface> &robot, Particle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature);
};

#endif /* MULTI_FAST_SLAM_H */

