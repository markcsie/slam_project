#include "../include/fast_slam2.h"

#include <iostream>

FastSlam2::FastSlam2(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const RobotModelInterface &robot, const MapModelInterface &map) :
particles_(num_particles), initial_w_(initial_w), robot_(&robot), map_(&map)
{
  assert(initial_x.size() == num_particles);
  for (size_t i = 0; i < num_particles; i++)
  {
    particles_[i].x_ = initial_x[i];
    particles_[i].w_ = initial_w_;
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
  //  std::cout << "u: " << u.transpose() << std::endl;
  const size_t num_measurements = features.rows();
  //  const size_t num_measurements = 0; // dead reckoning
  if (num_measurements == 0)
  {
    for (Particle &p : particles_)
    {
      p.cov_ = robot_->calculateRt(p.x_, u, p.cov_);
      p.x_ = robot_->samplePose(p.x_, u); // x_t ~ p(x_t| x_{t-1}, u_t)
    }
  }
  else
  {
//    std::cout << "ggg z: " << std::endl << features << std::endl;
    for (size_t i = 0; i < num_measurements; i++)
    {
      // implement the algorithm in Table 13.3
      for (Particle &p : particles_)
      {
        updateParticle(p, u, features.row(i));
        //  std::cout << "p.x_ " << p.x_.transpose() << std::endl;  //for debug
      }
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
//        std::cout << "ggg Utils::sampleDiscrete(weights) " << Utils::sampleDiscrete(weights) << std::endl;
        //      std::cout << "ggg new_particles[i].x_ " << new_particles[i].x_.transpose() << std::endl;
      }
      particles_ = new_particles;
//      std::cout << "ggg particles_.size() " << particles_.size() << std::endl;
    }
  }
}

// TODO: check if z_t is after applying u_t????
void FastSlam2::updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature)
{
  const int feature_id = feature[0];
  const Eigen::VectorXd z = feature.block(1, 0, feature.rows() - 1, 1);
//  std::cout << "ggg feature_id " << feature_id << std::endl;
//  std::cout << "ggg z " << z.transpose() << std::endl;
  auto iter = p.features_.find(feature_id);

  if (iter == p.features_.end()) // first time seeing the feature, do initialization 
  {
//    std::cout << "ggg new feature_id: " << feature_id << std::endl;

    p.x_ = robot_->samplePose(p.x_, u);
//    std::cout << "ggg p.x_ " << p.x_.transpose() << std::endl;
    p.features_[feature_id].mean_ = robot_->inverseMeasurement(map_, p.x_, z); // mean_t = h^{-1}(x_t, z_t))
//    std::cout << "ggg p.features_[feature_id].mean_ " << p.features_[feature_id].mean_.transpose() << std::endl;
    Eigen::MatrixXd H_m = robot_->jacobianFeature(map_, p.features_[feature_id].mean_, p.x_);
    p.features_[feature_id].covariance_ = H_m.inverse() * robot_->getQt() * H_m.inverse().transpose();
//    std::cout << "ggg p.features_[feature_id].covariance_ \n" << p.features_[feature_id].covariance_ << std::endl;
    p.w_ = initial_w_;
  }
  else
  {
//    std::cout << "ggg update feature_id: " << feature_id << std::endl;

    Eigen::VectorXd x_t = robot_->predictPose(p.x_, u); // g(x_{t-1}, u_t)
//    std::cout << "ggg x_t.transpose() " << x_t.transpose() << std::endl;
    Eigen::MatrixXd H_m = robot_->jacobianFeature(map_, p.features_[feature_id].mean_, x_t);
    
//    std::cout << "ggg H_m \n" << H_m << std::endl;

    Eigen::MatrixXd Q_j = robot_->getQt() + H_m * p.features_[feature_id].covariance_ * H_m.transpose();
//    std::cout << "ggg Q_j \n" << Q_j << std::endl;

    Eigen::MatrixXd H_x = robot_->jacobianPose(map_, p.features_[feature_id].mean_, x_t); // mean_t = h^{-1}(x_t, z_t))  
//    std::cout << "ggg H_x \n" << H_x << std::endl;
    Eigen::MatrixXd R_t = robot_->calculateRt(p.x_, u, p.cov_);
    
//    std::cout << "ggg R_t \n" << R_t << std::endl;
    p.cov_ = (H_x.transpose() * Q_j.inverse() * H_x + R_t.inverse()).inverse();
//    std::cout << "ggg Q_j.inverse() \n" << Q_j.inverse() << std::endl;
//    std::cout << "ggg R_t.inverse() \n" << R_t.inverse() << std::endl;
//    std::cout << "ggg (H_x.transpose() * Q_j.inverse() * H_x + R_t.inverse()) \n" << (H_x.transpose() * Q_j.inverse() * H_x + R_t.inverse()) << std::endl;

    Eigen::VectorXd z_bar = robot_->predictMeasurement(map_, p.features_[feature_id].mean_, x_t); // h(mean_{t-1}, x)
//    std::cout << "ggg z_bar.transpose() " << z_bar.transpose() << std::endl;
//    std::cout << "ggg z.transpose() " << z.transpose() << std::endl;
    Eigen::VectorXd mean_x = p.cov_ * H_x.transpose() * Q_j.inverse() * (z - z_bar) + x_t;
//    std::cout << "ggg mean_x.transpose() " << mean_x.transpose() << std::endl;
    
    // EKF update
    Eigen::MatrixXd K = p.features_[feature_id].covariance_ * H_m.transpose() * Q_j.inverse(); // Kalman gain

    p.x_ = Utils::sampleMultivariateGaussian(mean_x, p.cov_); // update pose
//    std::cout << "ggg p.x_.transpose() " << p.x_.transpose() << std::endl;
    Eigen::VectorXd z_hat = robot_->predictMeasurement(map_, p.features_[feature_id].mean_, p.x_); // h(mean_{t-1}, x)
    
    p.features_[feature_id].mean_ += K * (z - z_hat); // update feature mean
    p.features_[feature_id].covariance_ = (Eigen::MatrixXd::Identity(K.rows(), K.rows()) - K * H_m) * p.features_[feature_id].covariance_; // update feature covariance
    
    Eigen::MatrixXd L = H_x * R_t * H_x.transpose() + H_m * p.features_[feature_id].covariance_ * H_m.transpose() + robot_->getQt();
    double temp = (z - z_hat).transpose() * L.inverse() * (z - z_hat);
//    std::cout << "ggg temp " << temp << std::endl;
    p.w_ = (1 / std::sqrt(L.determinant() * 2 * M_PI)) * std::exp(temp / -2);
  }
}
