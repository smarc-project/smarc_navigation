#ifndef EKF_UTILS_HPP
#define EKF_UTILS_HPP

#include <ros/ros.h>
#include <eigen3/Eigen/Core>

namespace utils{

    void updateMatrixBlock(const Eigen::MatrixXd& sigma_in, Eigen::MatrixXd& sigma_out, int lm_num);

    void addLMtoFilter(Eigen::VectorXd &mu_hat, Eigen::MatrixXd &Sigma_hat, Eigen::Vector3d landmark, const std::tuple<double, double, double> &sigma_new);

    double angleLimit (double angle);
}

#endif // EKF_UTILS_HPP
