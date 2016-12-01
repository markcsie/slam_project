#include "../include/fast_slam2.h"

#include <iostream>

FastSlam2::FastSlam2(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const std::shared_ptr<const RobotModelInterface> &robot, const MapModelInterface &map) :
particles_(num_particles), initial_w_(initial_w), robot_(robot), robot_id_(robot_->getId()), map_(&map)
{
  assert(initial_x.size() >= 1);
  for (size_t i = 0; i < num_particles; i++)
  {
    particles_[i].x_[robot_id_] = initial_x[0];
    particles_[i].cov_ = initial_cov;
  }
}

FastSlam2::FastSlam2(const FastSlam2& other)
{
  // TODO:
}

FastSlam2::~FastSlam2()
{
}

size_t FastSlam2::getNumParticles()
{
  return particles_.size();
}

std::vector<Particle> FastSlam2::getParticles()
{
  return particles_;
}

Particle FastSlam2::getParticle(const size_t &i)
{
  return particles_[i];
}

void FastSlam2::process(const Eigen::VectorXd &u, const Eigen::MatrixXd &features)
{
  const size_t num_measurements = features.rows();
  //  const size_t num_measurements = 0; // dead reckoning
  if (num_measurements == 0)
  {
    for (Particle &p : particles_)
    {
      p.cov_ = robot_->calculateRt(p.x_[robot_id_], u, p.cov_);
      p.x_[robot_id_] = robot_->samplePose(p.x_[robot_id_], u); // x_t ~ p(x_t| x_{t-1}, u_t)
    }
  }
  else
  {
    //    std::cout << "ggg z: " << std::endl << features << std::endl;
    // implement the algorithm in Table 13.3
    std::vector<double> weights(particles_.size(), 1.0);
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
          weights[j] *= updateParticle(particles_[j], Eigen::VectorXd::Zero(u.rows()), features.row(i));
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
    //      std::cout << "ggg particles_.size() " << particles_.size() << std::endl;
  }
}

// TODO: check if z_t is after applying u_t????

double FastSlam2::updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature)
{
  double weight = initial_w_;

  const int feature_id = feature[0];
  const Eigen::VectorXd z = feature.block(1, 0, feature.rows() - 1, 1);
  auto iter = p.features_.find(feature_id);
  if (iter == p.features_.end()) // first time seeing the feature, do initialization 
  {
    p.x_[robot_id_] = robot_->samplePose(p.x_[robot_id_], u);
    p.features_[feature_id].mean_ = robot_->inverseMeasurement(map_, p.x_[robot_id_], z); // mean_t = h^{-1}(x_t, z_t))
    Eigen::MatrixXd H_m = robot_->jacobianFeature(map_, p.features_[feature_id].mean_, p.x_[robot_id_]);
    p.features_[feature_id].covariance_ = H_m.inverse() * robot_->getQt() * H_m.inverse().transpose();
    weight = initial_w_;
  }
  else
  {
    Eigen::VectorXd x_t = robot_->predictPose(p.x_[robot_id_], u); // g(x_{t-1}, u_t)
    Eigen::MatrixXd H_m = robot_->jacobianFeature(map_, p.features_[feature_id].mean_, x_t);

    Eigen::MatrixXd Q_j = robot_->getQt() + H_m * p.features_[feature_id].covariance_ * H_m.transpose();

    Eigen::MatrixXd H_x = robot_->jacobianPose(map_, p.features_[feature_id].mean_, x_t); // mean_t = h^{-1}(x_t, z_t))  
    Eigen::MatrixXd R_t = robot_->calculateRt(p.x_[robot_id_], u, p.cov_);
    p.cov_ = (H_x.transpose() * Q_j.inverse() * H_x + R_t.inverse()).inverse();
    
    Eigen::VectorXd z_bar = robot_->predictMeasurement(map_, p.features_[feature_id].mean_, x_t); // h(mean_{t-1}, x)
    Eigen::VectorXd z_bar_difference = z - z_bar;
    z_bar_difference[1] = std::remainder(z_bar_difference[1], 2 * M_PI);
    Eigen::VectorXd mean_x = p.cov_ * H_x.transpose() * Q_j.inverse() * z_bar_difference + x_t;

    // EKF update
    Eigen::MatrixXd K = p.features_[feature_id].covariance_ * H_m.transpose() * Q_j.inverse(); // Kalman gain

    p.x_[robot_id_] = Utils::sampleMultivariateGaussian(mean_x, p.cov_); // update pose
    Eigen::VectorXd z_hat = robot_->predictMeasurement(map_, p.features_[feature_id].mean_, p.x_[robot_id_]); // h(mean_{t-1}, x)
    Eigen::VectorXd z_hat_difference = z - z_hat;
    z_hat_difference[1] = std::remainder(z_hat_difference[1], 2 * M_PI);

    p.features_[feature_id].mean_ += K * z_hat_difference; // update feature mean
    p.features_[feature_id].covariance_ = (Eigen::MatrixXd::Identity(K.rows(), K.rows()) - K * H_m) * p.features_[feature_id].covariance_; // update feature covariance

    Eigen::MatrixXd L = H_x * R_t * H_x.transpose() + H_m * p.features_[feature_id].covariance_ * H_m.transpose() + robot_->getQt();
    double temp = z_hat_difference.transpose() * L.inverse() * z_hat_difference;
    weight = (1 / std::sqrt((2 * M_PI * L).determinant())) * std::exp(temp / -2);
  }

  return weight;
}
