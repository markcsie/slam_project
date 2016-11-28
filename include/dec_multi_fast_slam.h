#ifndef DEC_MULTI_FAST_SLAM_H
#define DEC_MULTI_FAST_SLAM_H

#include <stack>
#include <memory>

#include <Eigen/Dense>

#include "particle.h"
#include "robot_models/mobile_robot_2dmodel.h"
#include "map_models/interface_map_model.h"


// Decentralized Multi-robot SLAM with unknown initial poses
class DecMultiFastSlam
{
public:
  DecMultiFastSlam(const size_t &num_particles, const Eigen::VectorXd &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const std::vector<std::shared_ptr<const RobotModelInterface>> &robots, const MapModelInterface &map);
  DecMultiFastSlam(const DecMultiFastSlam& other);
  virtual ~DecMultiFastSlam();

  size_t getNumParticles();
  std::vector<Particle> getParticles();
  Particle getParticle(const size_t &i);
  void process(const std::vector<Eigen::VectorXd> &u, const std::vector<Eigen::MatrixXd> &features);
private:
  std::vector<std::shared_ptr<const RobotModelInterface>> robots_;
  
  std::vector<std::shared_ptr<const RobotModelInterface>> virtual_robots_;
  std::vector<std::stack<Eigen::VectorXd>> virtual_robots_u_;
  std::vector<std::stack<Eigen::MatrixXd>> virtual_robots_z_;
  
  std::shared_ptr<const MapModelInterface> map_;

  double initial_w_;
  std::vector<Particle> particles_;

  int getRobotIndex(const int &id);
  
  std::vector<double> updateRobot(const std::shared_ptr<const RobotModelInterface> &robot, const Eigen::VectorXd &u, const Eigen::MatrixXd &features);

  double updateParticle(const std::shared_ptr<const RobotModelInterface> &robot, Particle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature);
  
};

#endif /* DEC_MULTI_FAST_SLAM_H */

