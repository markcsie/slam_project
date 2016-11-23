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
  assert(initial_x.size() == robots_.size());
  for (size_t i = 0; i < num_particles; i++)
  {
    for (size_t j = 0; j < initial_x.size(); j++)
    {
      particles_[i].x_[robots_[j]->getId()] = initial_x[j];
    }
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
  assert(u.size() == robots_.size() && u.size() == features.size());

  std::vector<double> weights;
  for (size_t i = 0; i < robots_.size(); i++)
  {
    weights = updateRobot(robots_[i], u[i], features[i]);
  }

  // resampling
  assert(weights.size() == particles_.size());
  std::vector<MultiRobotParticle> new_particles(particles_.size());
  for (size_t i = 0; i < particles_.size(); i++)
  {
    new_particles[i] = particles_[Utils::sampleDiscrete(weights)];
  }
  particles_ = new_particles;
}

std::vector<double> MultiFastSlam::updateRobot(const std::shared_ptr<const RobotModelInterface> &robot, const Eigen::VectorXd &u, const Eigen::MatrixXd &features)
{
  std::vector<double> weights(particles_.size(), 1.0);

  const size_t num_measurements = features.rows();
  const int robot_id = robot->getId();
  if (num_measurements == 0)
  {
    for (MultiRobotParticle &p : particles_)
    {
      p.x_[robot_id] = robot->samplePose(p.x_[robot_id], u); // x_t ~ p(x_t| x_{t-1}, u_t)
    }
  }
  else
  {
    for (size_t i = 0; i < num_measurements; i++)
    {
      for (size_t j = 0; j < particles_.size(); j++)
      {
        if (i == 0)
        {
          weights[j] *= updateParticle(robot, particles_[j], u, features.row(i));
        }
        else
        {
          weights[j] *= updateParticle(robot, particles_[j], Eigen::VectorXd::Zero(u.rows(), u.cols()), features.row(i));
        }
      }
    }
  }

  return weights;
}

// TODO: check if z_t is after applying u_t????
double MultiFastSlam::updateParticle(const std::shared_ptr<const RobotModelInterface> &robot, MultiRobotParticle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature)
{
  double weight = initial_w_;

  const int robot_id = robot->getId();
  const int feature_id = feature[0];
  const Eigen::VectorXd z = feature.block(1, 0, feature.rows() - 1, 1);
  auto iter = p.features_.find(feature_id);

  p.x_[robot_id] = robot->samplePose(p.x_[robot_id], u);

  if (iter == p.features_.end()) // first time seeing the feature, do initialization 
  {
    p.features_[feature_id].mean_ = robot->inverseMeasurement(map_, p.x_[robot_id], z); // mean_t = h^{-1}(x_t, z_t))
    Eigen::MatrixXd H = robot->jacobianFeature(map_, p.features_[feature_id].mean_, p.x_[robot_id]);

    p.features_[feature_id].covariance_ = H.inverse() * robot->getQt() * H.inverse().transpose();
    weight = initial_w_;
  }
  else
  {
    Eigen::VectorXd z_hat = robot->predictMeasurement(map_, p.features_[feature_id].mean_, p.x_[robot_id]); // h(mean_{t-1}, x)
    Eigen::MatrixXd H = robot->jacobianFeature(map_, p.features_[feature_id].mean_, p.x_[robot_id]);
    Eigen::MatrixXd Q = H * p.features_[feature_id].covariance_ * H.transpose() + robot->getQt();
    Eigen::MatrixXd K = p.features_[feature_id].covariance_ * H.transpose() * Q.inverse(); // Kalman gain
    p.features_[feature_id].mean_ += K * (z - z_hat); // update feature mean
    p.features_[feature_id].covariance_ = (Eigen::MatrixXd::Identity(K.rows(), K.rows()) - K * H) * p.features_[feature_id].covariance_; // update feature covariance
    double temp = (z - z_hat).transpose() * Q.inverse() * (z - z_hat);
    weight = (1 / std::sqrt((2 * M_PI * Q).determinant())) * std::exp(temp / -2);
  }

  return weight;
}