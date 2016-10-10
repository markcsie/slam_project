#include "fast_slam2.h"

FastSlam2::FastSlam2(const size_t &num_particles, const double &initial_w) :
particles_(num_particles), initial_w_(initial_w)
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
  for (size_t k = 0; k < particles_.size(); k++)
  {
    updateParticle(particles_[k], u, z);
  }
  // resampling
  std::vector<double> weights(particles_.size());
  for (size_t i = 0; i < weights.size(); i++) 
  {
    weights[i] = particles_[i].w_;
  }
  std::discrete_distribution<int> sampler(weights.begin(), weights.end());
  std::vector<Particle> new_particles(particles_.size());
  for (size_t i = 0; i < particles_.size(); i++)
  {
    new_particles[i] = particles_[sampler(generator)];
  }
  particles_ = new_particles;
}

Eigen::VectorXd FastSlam2::samplePose(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
  // x_t ~ p(x_t| x_{t-1}, u_t)
  Eigen::VectorXd x_next;
  return x_next;
}

Eigen::VectorXd FastSlam2::samplePoseGaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
{
  // x ~ N(mean, covariance)
  Eigen::VectorXd x;
  return x;
}

Eigen::VectorXd FastSlam2::predictPose(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
  // g(x_{t-1}, u_t)
  Eigen::VectorXd x_next;
  return x_next;
}

Eigen::VectorXd FastSlam2::predictMeasurement(const Eigen::VectorXd &mean, const Eigen::VectorXd &x)
{
  // h(mean_{t-1}, x)
  Eigen::VectorXd z;
  return z;
}

Eigen::VectorXd FastSlam2::inverseMeasurement(const Eigen::VectorXd &x, const Eigen::VectorXd &z)
{
  Eigen::VectorXd mean;
  return mean;
}

Eigen::MatrixXd FastSlam2::jacobianPose(const Eigen::VectorXd &mean, const Eigen::VectorXd &x)
{
  Eigen::MatrixXd H_x;
  return H_x;
}

Eigen::MatrixXd FastSlam2::jacobianFeature(const Eigen::VectorXd &mean, const Eigen::VectorXd &x)
{
  Eigen::MatrixXd H_m;
  return H_m;
}

void FastSlam2::updateParticle(Particle &p, const Eigen::VectorXd &u, const Eigen::MatrixXd &z)
{
  //    size_t correspondence =;
  // sample pose
  samplePose(p.x_, u);

  const size_t num_measurements = z.rows();
  for (size_t m = 0; m < num_measurements; m++) // for each measurement, update its corresponding kalman filter
  {
    const int feature_id = z(m, 0);
    auto iter = p.features_.find(feature_id);
    Eigen::MatrixXd Q_t; // TODO: Q_t parameters
    Eigen::MatrixXd R_t; // TODO: R_t parameters
    if (iter == p.features_.end()) // first time seeing the feature, do initialization 
    {
      p.x_ = samplePose(p.x_, u);
      p.features_[feature_id].mean_ = inverseMeasurement(p.x_, z);
      Eigen::MatrixXd H_m = jacobianFeature(p.features_[feature_id].mean_, p.x_);
      p.features_[feature_id].covariance_ = H_m.inverse().transpose() * Q_t * H_m.inverse();
      p.w_ = initial_w_;
    }
    else
    {
      Eigen::VectorXd x_t = predictPose(p.x_, u);
      Eigen::MatrixXd H_m = jacobianFeature(p.features_[feature_id].mean_, x_t);

      Eigen::MatrixXd Q_j = Q_t + H_m * p.features_[feature_id].covariance_ * H_m.transpose(); // TODO: what is Q_t

      Eigen::MatrixXd H_x = jacobianPose(p.features_[feature_id].mean_, x_t);
      Eigen::MatrixXd covariance_x = (H_x.transpose() * Q_j.inverse() * H_x + R_t.inverse()).inverse();
      Eigen::VectorXd z_bar = predictMeasurement(p.features_[feature_id].mean_, x_t);
      Eigen::VectorXd mean_x = covariance_x * H_x.transpose() * Q_j.inverse() * (z - z_bar) + x_t;

      // EKF update
      Eigen::MatrixXd K = p.features_[feature_id].covariance_ * H_m.transpose() * Q_j.inverse(); // Kalman gain

      p.x_ = samplePoseGaussian(mean_x, covariance_x); // update pose
      Eigen::VectorXd z_hat = predictMeasurement(p.features_[feature_id].mean_, p.x_);
      p.features_[feature_id].mean_ += K * (z - z_hat); // update feature mean

      Eigen::MatrixXd L = H_x * R_t * H_x.transpose() + H_m * p.features_[feature_id].covariance_ * H_m.transpose() + Q_t;
      p.features_[feature_id].covariance_ = (Eigen::MatrixXd::Identity(K.rows(), K.rows()) - K * H_m) * p.features_[feature_id].covariance_; // update feature covariance
      Eigen::MatrixXd temp = (z - z_hat).transpose() * L.inverse() * (z - z_hat);
      p.w_ = (1 / std::sqrt(L.determinant() * 2 * M_PI)) * std::exp(temp(0, 0) / -2);
    }
  }
}
