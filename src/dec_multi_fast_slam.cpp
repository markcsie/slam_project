#include "../include/dec_multi_fast_slam.h"

#include <iostream>

#include "utils/eigenmvn.h"

DecMultiFastSlam::DecMultiFastSlam(const size_t &num_particles, const Eigen::VectorXd &initial_x, const Eigen::MatrixXd &initial_cov, const double &initial_w, const std::vector<std::shared_ptr<const RobotModelInterface>> &robots, const MapModelInterface &map) :
particles_(num_particles), initial_w_(initial_w), robots_(robots), map_(&map)
{
  // Initialize the data stack for virtual robots
  for (size_t i = 0; i < robots_.size(); i++)
  {
    virtual_robots_u_[-robots_[i]->getId()] = std::stack<Eigen::VectorXd>();
    virtual_robots_z_[-robots_[i]->getId()] = std::stack<Eigen::MatrixXd>();
  }

  for (size_t i = 0; i < num_particles; i++)
  {
    particles_[i].x_[robots_[0]->getId()] = initial_x;
    particles_[i].cov_ = initial_cov;
  }

}

DecMultiFastSlam::DecMultiFastSlam(const DecMultiFastSlam& other)
{
  // TODO:
}

DecMultiFastSlam::~DecMultiFastSlam()
{
}

size_t DecMultiFastSlam::getNumParticles()
{
  return particles_.size();
}

std::vector<Particle> DecMultiFastSlam::getParticles()
{
  return particles_;
}

Particle DecMultiFastSlam::getParticle(const size_t &i)
{
  return particles_[i];
}

int DecMultiFastSlam::getRobotIndex(const int &id)
{
  for (size_t i = 0; i < robots_.size(); i++)
  {
    if (id == robots_[i]->getId())
    { // this observation is a robot
      return i;
    }
  }
  return -1;
}

void DecMultiFastSlam::process(const std::vector<Eigen::VectorXd> &u, const std::vector<Eigen::MatrixXd> &features)
{
  //  assert(u.size() == robots_.size() && u.size() == features.size());

  std::vector<double> forward_weights(particles_.size(), 1.0);
  for (size_t i = 0; i < u.size(); i++)
  {
    // This robot has not been incorporated into the map => Store the data.  
    if (particles_[0].x_.find(robots_[i]->getId()) == particles_[0].x_.end())
    {
      const int virtual_robot_id = -robots_[i]->getId();
      virtual_robots_u_[virtual_robot_id].push(u[i]);
      virtual_robots_z_[virtual_robot_id].push(features[i]);
    }
    else
    {
      // forward robots
      forward_weights = updateRobot(robots_[i], u[i], features[i], false);
    }
  }

  // backward robots
  std::vector<double> backward_weights(particles_.size(), 1.0);
  for (const int virtual_robot_id : virtual_robots_id_) // new virtual robots might be added during backward updating
  {
    // TODO: if the stack is empty
    // TODO: -u? check if z is aligned???
    // TODO: Workaround
    int virtual_robot_index;
    for (size_t i = 0; i < robots_.size(); i++)
    {
      if (-virtual_robot_id == robots_[i]->getId())
      {
        virtual_robot_index = i;
        break;
      }
    }
    
    if (virtual_robots_u_[virtual_robot_id].size() > 0 && virtual_robots_z_[virtual_robot_id].size() > 0)
    {
      backward_weights = updateRobot(robots_[virtual_robot_index], -virtual_robots_u_[virtual_robot_id].top(), virtual_robots_z_[virtual_robot_id].top(), true);
      virtual_robots_z_[virtual_robot_id].pop();
      virtual_robots_u_[virtual_robot_id].pop();
    }
  }

  // resampling
  std::vector<double> weights(particles_.size(), 1.0);
  for (size_t i = 0; i < weights.size(); i++)
  {
    weights[i] = forward_weights[i] * backward_weights[i];
  }

  std::vector<Particle> new_particles(particles_.size());
  for (size_t i = 0; i < particles_.size(); i++)
  {
    new_particles[i] = particles_[Utils::sampleDiscrete(weights)];
  }
  particles_ = new_particles;
}

std::vector<double> DecMultiFastSlam::updateRobot(const std::shared_ptr<const RobotModelInterface> &robot, const Eigen::VectorXd &u, const Eigen::MatrixXd &features, const bool &backward)
{
  std::vector<double> weights(particles_.size(), 1.0);

  const size_t num_measurements = features.rows();
  int robot_id = robot->getId();
  if (backward)
  {
    robot_id = -robot_id;
  }
  
  if (num_measurements == 0)
  {
    for (Particle &p : particles_)
    {
      p.x_[robot_id] = robot->samplePose(p.x_[robot_id], u); // x_t ~ p(x_t| x_{t-1}, u_t)
    }
  }
  else
  {
    for (size_t i = 0; i < num_measurements; i++)
    {
      const Eigen::VectorXd feature = features.row(i);
      const int id = feature[0];
      int robot_index = getRobotIndex(id);
      if (robot_index >= 0) // The feature is a robot
      {
        if (particles_[0].x_.find(id) == particles_[0].x_.end())
        {
          virtual_robots_id_.push_back(-id);
          Eigen::VectorXd z = feature.block(1, 0, feature.rows() - 1, 1);
          for (Particle &p : particles_)
          {
            // initialize the state of the causal and virtual robot
            Eigen::VectorXd position = robot->sampleInverseMeasurement(map_, p.x_[robot_id], z); // mean_t = h^{-1}(x_t, z_t))
            // TODO: dimension
            Eigen::VectorXd pose(robot->getDim());
            pose << position[0], position[1], Utils::sampleUniform(-M_PI, M_PI); // Random orientation for each particle
            p.x_[id] = pose;
            p.x_[-id] = pose;
          }
        }
        // ignore the measurement if the robot has been seen before
      }
      else // The feature is a landmark
      {
        // forward robots
        for (size_t j = 0; j < particles_.size(); j++)
        {
          if (i == 0)
          {
            weights[j] *= updateParticle(robot, particles_[j], u, feature, backward);
          }
          else
          {
            weights[j] *= updateParticle(robot, particles_[j], Eigen::VectorXd::Zero(u.rows()), feature, backward);
          }
        }
      }
    }
  }
  return weights;
}

// TODO: check if z_t is after applying u_t????

double DecMultiFastSlam::updateParticle(const std::shared_ptr<const RobotModelInterface> &robot, Particle &p, const Eigen::VectorXd &u, const Eigen::VectorXd &feature, const bool &backward)
{
  double weight = initial_w_;

  int robot_id = robot->getId();
  if (backward)
  {
    robot_id = -robot_id;
  }
    
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