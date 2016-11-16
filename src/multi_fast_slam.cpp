#include "../include/multi_fast_slam.h"

#include <iostream>

#include "utils/eigenmvn.h"

MultiFastSlam::MultiFastSlam(const size_t &num_particles, const std::vector<Eigen::VectorXd> &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const std::vector<RobotModelInterface> &robots, const MapModelInterface &map) :
particles_(num_particles), initial_w_(initial_w), map_(&map)
{
  for (const RobotModelInterface & r : robots)
  {
    robots_.push_back(std::shared_ptr<const RobotModelInterface>(&r));
  }
  assert(initial_x.size() == robots_.size()); // TODO: this is different from single slam
  for (size_t i = 0; i < num_particles; i++)
  {
    for (size_t j = 0; j < initial_x.size(); j++) {
      particles_[i].x_[robots_[j]->getId()] = initial_x[j];
    }
    particles_[i].w_ = initial_w_;
    particles_[i].cov_ = initial_cov;
  }

}

MultiFastSlam::MultiFastSlam(const MultiFastSlam& other)
{
  // TODO:
}

MultiFastSlam::~MultiFastSlam()
{
}

size_t MultiFastSlam::getNumParticles() 
{
  return particles_.size();
}

std::vector<MultiRobotParticle> MultiFastSlam::getParticles()
{
  return particles_;
}

MultiRobotParticle MultiFastSlam::getParticle(const size_t &i)
{
  return particles_[i];
}

void MultiFastSlam::process(const std::vector<Eigen::VectorXd> &u, const std::vector<Eigen::MatrixXd> &features)
{
}

//// TODO: check if z_t is after applying u_t????
//void MultiFastSlam::updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature)
//{
////  const int feature_id = feature[0];
////  const Eigen::VectorXd z = feature.block(1, 0, feature.rows() - 1, 1);
////  auto iter = p.features_.find(feature_id);
////
////  std::cout << "ggg p.x_ " << p.x_.transpose() << std::endl;
////  p.x_ = samplePose(p.x_, u);
////  std::cout << "ggg p.x_ " << p.x_.transpose() << std::endl;
////  
////  if (iter == p.features_.end()) // first time seeing the feature, do initialization 
////  {
////    p.features_[feature_id].mean_ = inverseMeasurement(p.x_, z);
////    Eigen::MatrixXd H = jacobianFeature(p.features_[feature_id].mean_, p.x_);
////    p.features_[feature_id].covariance_ = H.inverse() * robot_->getQt() * H.inverse().transpose();
////    p.w_ = initial_w_;
////  }
////  else
////  {
////    Eigen::VectorXd z_hat = predictMeasurement(p.features_[feature_id].mean_, p.x_);
////    Eigen::MatrixXd H = jacobianFeature(p.features_[feature_id].mean_, p.x_);
////    Eigen::MatrixXd Q = H * p.features_[feature_id].covariance_ * H.transpose() + robot_->getQt();
////    Eigen::MatrixXd K = p.features_[feature_id].covariance_ * H.transpose() * Q.inverse(); // Kalman gain
////    p.features_[feature_id].mean_ += K * (z - z_hat); // update feature mean
////    p.features_[feature_id].covariance_ = (Eigen::MatrixXd::Identity(K.rows(), K.rows()) - K * H) * p.features_[feature_id].covariance_; // update feature covariance
////    double temp = (z - z_hat).transpose() * Q.inverse() * (z - z_hat);
////    p.w_ = (1 / std::sqrt(Q.determinant() * 2 * M_PI)) * std::exp(temp / -2);
////  }
//}
