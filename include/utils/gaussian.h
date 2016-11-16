#ifndef GAUSSIAN_H
#define GAUSSIAN_H

struct Gaussian
{
  Eigen::VectorXd mean_;
  Eigen::MatrixXd covariance_;
};

#endif /* GAUSSIAN_H */

