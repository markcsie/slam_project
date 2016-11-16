#ifndef FAST_SLAM_H
#define FAST_SLAM_H

#include <unordered_map>
#include <memory>

#include <Eigen/Dense>

#include "particle.h"
#include "robot_models/mobile_robot_2dmodel.h"
#include "map_models/interface_map_model.h"

class FastSlam
{
public:
  FastSlam(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const RobotModelInterface &robot, const MapModelInterface &map);
  FastSlam(const FastSlam& other);
  virtual ~FastSlam();

  size_t getNumParticles();
  std::vector<Particle> getParticles();
  Particle getParticle(const size_t &i);
  void process(const Eigen::VectorXd &u, const Eigen::MatrixXd &features);
private:
  std::shared_ptr<const RobotModelInterface> robot_;
  std::shared_ptr<const MapModelInterface> map_;

  double initial_w_;
  std::vector<Particle> particles_;

  void updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature);

  Eigen::VectorXd samplePose(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
  Eigen::VectorXd predictMeasurement(const Eigen::VectorXd &mean, const Eigen::VectorXd &x)const;
  Eigen::VectorXd inverseMeasurement(const Eigen::VectorXd &x, const Eigen::VectorXd &z) const;
  Eigen::MatrixXd jacobianFeature(const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const;
  
  bool dead_reckoning_;
};

#endif /* FAST_SLAM_H */

