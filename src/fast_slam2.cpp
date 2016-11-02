#include "../include/fast_slam2.h"

#include <iostream>

#include "utils/eigenmvn.h"

FastSlam2::FastSlam2(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const double &initial_w, const RobotModelInterface &robot, const MapModelInterface &map) :
particles_(num_particles), initial_w_(initial_w), robot_(&robot), map_(&map)
{
  assert(initial_x.size() == num_particles);
  for (size_t i = 0; i < num_particles; i++)
  {
    particles_[i].x_ = initial_x[i];
    particles_[i].w_ = initial_w_;
  }
  
  // for debugging
  dead_reckoning_ = false;
}

FastSlam2::FastSlam2(const FastSlam2& other)
{
  // TODO:
}

FastSlam2::~FastSlam2()
{
}

Particle FastSlam2::getParticle(const size_t &i) 
{
  return particles_[i];
}

void FastSlam2::process(const Eigen::VectorXd &u, const Eigen::MatrixXd &z)
{
  // TODO: This is just testing
  // Eigen::VectorXd mean(2);
  // mean << 100, 1;
  // Eigen::MatrixXd cov(2, 2);
  // cov << 1, 0,
  //         0, 1;
  // std::cout << sampleMultivariateGaussian(mean, cov) << std::endl;
  // std::cout << sampleMultivariateGaussian(mean, cov) << std::endl;

  // implement the algorithm in Table 13.3
  std::cout << "u: " << u.transpose() << std::endl;
  for (int i = 0; i < z.rows(); i++)
  {
    std::cout << "z: " << z << std::endl;
  }


  for (auto &p : particles_)
  {
    updateParticle(p, u, z);
    std::cout << "p.x_ " << p.x_.transpose() << std::endl;
  }

  if (!dead_reckoning_)
  {
    // resampling
    std::vector<double> weights(particles_.size());
    for (size_t i = 0; i < weights.size(); i++)
    {
      weights[i] = particles_[i].w_;
    }
    std::vector<Particle> new_particles(particles_.size());
    for (size_t i = 0; i < particles_.size(); i++)
    {
      new_particles[i] = particles_[Utils::sampleDiscrete(weights)];
      std::cout << "new_particles[i].x_ " << new_particles[i].x_.transpose() << std::endl;
    }
    particles_ = new_particles;
  }
}

Eigen::MatrixXd FastSlam2::calculateRt(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const
{
  // R_t = V_t M_t V_t^{T}
  return robot_->calculateRt(x, u);
}

Eigen::VectorXd FastSlam2::samplePose(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const
{
  // x_t ~ p(x_t| x_{t-1}, u_t)
  return robot_->samplePose(x, u);
}

Eigen::VectorXd FastSlam2::sampleMultivariateGaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance) const
{
  // x ~ N(mean, covariance)
  Eigen::EigenMultivariateNormal<double> norm(mean, covariance);
  return norm.samples(1);
}

Eigen::VectorXd FastSlam2::predictPose(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const
{
  // g(x_{t-1}, u_t)
  return robot_->predictPose(x, u);
}

Eigen::VectorXd FastSlam2::predictMeasurement(const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const
{
  // h(mean_{t-1}, x)
  return robot_->predictMeasurement(map_, mean, x);
}

Eigen::VectorXd FastSlam2::inverseMeasurement(const Eigen::VectorXd &x, const Eigen::VectorXd &z) const
{
  // mean_t = h^{-1}(x_t, z_t))
  return robot_->inverseMeasurement(map_, x, z);
}

Eigen::MatrixXd FastSlam2::jacobianPose(const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const
{
  // mean_t = h^{-1}(x_t, z_t))  
  return robot_->jacobianPose(map_, mean, x);
}

Eigen::MatrixXd FastSlam2::jacobianFeature(const Eigen::VectorXd &mean, const Eigen::VectorXd &x) const
{
  return robot_->jacobianFeature(map_, mean, x);
}

void FastSlam2::updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::MatrixXd &features)
{
  // sample pose
  const size_t num_measurements = features.rows();
  // motion update only
  if (num_measurements == 0 || dead_reckoning_)
  {
    p.x_ = samplePose(p.x_, u);
  }
  else
  {
    for (size_t m = 0; m < num_measurements; m++) // for each measurement, update its corresponding kalman filter
    {
      const Eigen::VectorXd feature = features.row(m).transpose();
      const int feature_id = feature[0];
      const Eigen::VectorXd z = feature.block(1, 0, feature.rows() - 1, 1);
      auto iter = p.features_.find(feature_id);

      if (iter == p.features_.end()) // first time seeing the feature, do initialization 
      {
        std::cout << "new feature_id: " << feature_id << std::endl;

        p.x_ = samplePose(p.x_, u);
        p.features_[feature_id].mean_ = inverseMeasurement(p.x_, z);
        Eigen::MatrixXd H_m = jacobianFeature(p.features_[feature_id].mean_, p.x_);
        p.features_[feature_id].covariance_ = H_m.inverse().transpose() * robot_->getQt() * H_m.inverse();
        p.w_ = initial_w_;

      }
      else
      {
        std::cout << "update feature_id: " << feature_id << std::endl;

        Eigen::VectorXd x_t = predictPose(p.x_, u);
        std::cout << "ggg x_t.transpose() " << x_t.transpose() << std::endl;
        Eigen::MatrixXd H_m = jacobianFeature(p.features_[feature_id].mean_, x_t);
        std::cout << "ggg H_m \n" << H_m << std::endl;

        Eigen::MatrixXd Q_j = robot_->getQt() + H_m * p.features_[feature_id].covariance_ * H_m.transpose();
        std::cout << "ggg Q_j \n" << Q_j << std::endl;

        Eigen::MatrixXd H_x = jacobianPose(p.features_[feature_id].mean_, x_t);
        std::cout << "ggg H_x \n" << H_x << std::endl;
        Eigen::MatrixXd R_t = calculateRt(p.x_, u);
        std::cout << "ggg R_t \n" << R_t << std::endl;
        Eigen::MatrixXd covariance_x = (H_x.transpose() * Q_j.inverse() * H_x + R_t.inverse()).inverse();
        std::cout << "ggg Q_j.inverse() \n" << Q_j.inverse() << std::endl;
        std::cout << "ggg R_t.inverse() \n" << R_t.inverse() << std::endl;
        std::cout << "ggg covariance_x \n" << covariance_x << std::endl;
        
        Eigen::VectorXd z_bar = predictMeasurement(p.features_[feature_id].mean_, x_t);
        std::cout << "ggg z_bar.transpose() " << z_bar.transpose() << std::endl;
        Eigen::VectorXd mean_x = covariance_x * H_x.transpose() * Q_j.inverse() * (z - z_bar) + x_t;

        // EKF update
        Eigen::MatrixXd K = p.features_[feature_id].covariance_ * H_m.transpose() * Q_j.inverse(); // Kalman gain

        p.x_ = sampleMultivariateGaussian(mean_x, covariance_x); // update pose
        Eigen::VectorXd z_hat = predictMeasurement(p.features_[feature_id].mean_, p.x_);
        p.features_[feature_id].mean_ += K * (z - z_hat); // update feature mean

        Eigen::MatrixXd L = H_x * R_t * H_x.transpose() + H_m * p.features_[feature_id].covariance_ * H_m.transpose() + robot_->getQt();
        p.features_[feature_id].covariance_ = (Eigen::MatrixXd::Identity(K.rows(), K.rows()) - K * H_m) * p.features_[feature_id].covariance_; // update feature covariance
        Eigen::MatrixXd temp = (z - z_hat).transpose() * L.inverse() * (z - z_hat);
        p.w_ = (1 / std::sqrt(L.determinant() * 2 * M_PI)) * std::exp(temp(0, 0) / -2);
      }
    }
  }
}
