#include "fast_slam2.h"

FastSlam2::FastSlam2(size_t num_particles) :
num_particles_(num_particles), particles_(num_particles)
{

}

FastSlam2::FastSlam2(const FastSlam2& orig)
{
}

FastSlam2::~FastSlam2()
{
}

void FastSlam2::process(const Eigen::VectorXd &u, const Eigen::MatrixXd &z)
{
  // implement the algorithm in Table 13.3
  for (size_t k = 0; k < num_particles_; k++)
  {
    updateParticle(particles_[k], u, z);
  }
  // resampling
  std::vector<Particle> new_particles;
  for (size_t i = 0; i < num_particles_; i++) {
  
  }
  particles_ = new_particles;
}

void FastSlam2::updateParticle(const Particle p, const Eigen::VectorXd &u, const Eigen::MatrixXd &z)
{
  //    size_t correspondence =;
  const size_t num_measurements = z.rows();
  for (size_t m = 0; m < num_measurements; m++) // for each measurement, update its corresponding kalman filter
  {
    const int feature_id = z(m, 0);
    auto iter = p.features.find(feature_id);
    if (iter == p.features.end()) // first time seeing the feature, do initialization 
    {
      // features_[feature_id].mean = // mean
      // features_[feature_id].covariance = // covariance
    } else
    {
      // EKF update
    }
  }
}
