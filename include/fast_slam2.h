#ifndef FAST_SLAM2_H
#define FAST_SLAM2_H

#include <cstddef>
#include <vector>
#include <unordered_map>
#include <utility>
#include <Eigen/Dense>

struct Gaussian
{
  Eigen::VectorXd mean;
  Eigen::MatrixXd covariance;
};

struct Particle
{
  Eigen::VectorXd x;
  std::unordered_map<int, Gaussian> features; // key = subject # value Gaussian(mean, covariance)
  double w; // importance weight
};

class FastSlam2
{
public:
  FastSlam2(size_t num_particles);
  FastSlam2(const FastSlam2& orig);
  virtual ~FastSlam2();
  
  void process(const Eigen::VectorXd &u, const Eigen::MatrixXd &z);
private:
  size_t num_particles_;
  size_t num_features_;
  
  std::vector<Particle> particles_;
  
  void updateParticle(const Particle p, const Eigen::VectorXd &u, const Eigen::MatrixXd &z);
};

#endif /* FAST_SLAM2_H */

