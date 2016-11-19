#include "../include/fast_slam.h"

#include <iostream>

FastSlam::FastSlam(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const RobotModelInterface &robot, const MapModelInterface &map) :
particles_(num_particles), initial_w_(initial_w), robot_(&robot), map_(&map)
{
  assert(initial_x.size() >= 1);
  for (size_t i = 0; i < num_particles; i++)
  {
    particles_[i].x_ = initial_x[0];
    particles_[i].cov_ = initial_cov;
  }
}

FastSlam::FastSlam(const FastSlam& other)
{
  // TODO:
}

FastSlam::~FastSlam()
{
}

size_t FastSlam::getNumParticles() 
{
  return particles_.size();
}

std::vector<Particle> FastSlam::getParticles()
{
  return particles_;
}

Particle FastSlam::getParticle(const size_t &i)
{
  return particles_[i];
}

void FastSlam::process(const Eigen::VectorXd &u, const Eigen::MatrixXd &features)
{
  const size_t num_measurements = features.rows();
  //  const size_t num_measurements = 0; // dead reckoning
  if (num_measurements == 0)
  {
    for (Particle &p : particles_)
    {
      //      std::cout << "ggg p.x_ " << p.x_.transpose() << std::endl;
      p.x_ = robot_->samplePose(p.x_, u); // x_t ~ p(x_t| x_{t-1}, u_t)
      //      std::cout << "ggg p.x_ " << p.x_.transpose() << std::endl;
    }
  }
  else
  {
    // implement the algorithm in Table 13.1
    std::vector<double> weights(particles_.size());
    for (size_t i = 0; i < num_measurements; i++)
    {
      for (size_t j = 0; j < particles_.size(); j++)
      {
        if (i == 0)
        {
          weights[j] *= updateParticle(particles_[j], u, features.row(i));
        }
        else
        {
          weights[j] *= updateParticle(particles_[j], u.Zero(u.rows(), u.cols()), features.row(i));
        }
      }
    }

    // resampling
    std::vector<Particle> new_particles(particles_.size());
    for (size_t i = 0; i < particles_.size(); i++)
    {
      new_particles[i] = particles_[Utils::sampleDiscrete(weights)];
      //        std::cout << "ggg Utils::sampleDiscrete(weights) " << Utils::sampleDiscrete(weights) << std::endl;
      //      std::cout << "ggg new_particles[i].x_ " << new_particles[i].x_.transpose() << std::endl;
    }
    particles_ = new_particles;
  }
}

// TODO: check if z_t is after applying u_t????
double FastSlam::updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature)
{
  double weight = initial_w_;

  const int feature_id = feature[0];
  const Eigen::VectorXd z = feature.block(1, 0, feature.rows() - 1, 1);

  p.x_ = robot_->samplePose(p.x_, u); // x_t ~ p(x_t| x_{t-1}, u_t)

  auto iter = p.features_.find(feature_id);
  if (iter == p.features_.end()) // first time seeing the feature, do initialization 
  {
    p.features_[feature_id].mean_ = robot_->inverseMeasurement(map_, p.x_, z); // mean_t = h^{-1}(x_t, z_t))
    Eigen::MatrixXd H = robot_->jacobianFeature(map_, p.features_[feature_id].mean_, p.x_);
    p.features_[feature_id].covariance_ = H.inverse() * robot_->getQt() * H.inverse().transpose();
    weight = initial_w_;
  }
  else
  {
    Eigen::VectorXd z_hat = robot_->predictMeasurement(map_, p.features_[feature_id].mean_, p.x_); // h(mean_{t-1}, x)
    Eigen::MatrixXd H = robot_->jacobianFeature(map_, p.features_[feature_id].mean_, p.x_);
    Eigen::MatrixXd Q = H * p.features_[feature_id].covariance_ * H.transpose() + robot_->getQt();
    Eigen::MatrixXd K = p.features_[feature_id].covariance_ * H.transpose() * Q.inverse(); // Kalman gain
    p.features_[feature_id].mean_ += K * (z - z_hat); // update feature mean
    p.features_[feature_id].covariance_ = (Eigen::MatrixXd::Identity(K.rows(), K.rows()) - K * H) * p.features_[feature_id].covariance_; // update feature covariance
    double temp = (z - z_hat).transpose() * Q.inverse() * (z - z_hat);
//    std::cout << "feature weight " << (1 / std::sqrt((2 * M_PI * Q).determinant())) * std::exp(temp / -2) << std::endl;
    weight = (1 / std::sqrt((2 * M_PI * Q).determinant())) * std::exp(temp / -2);
//    std::cout << "weight " << weight << std::endl;
  }
  return weight;
}
