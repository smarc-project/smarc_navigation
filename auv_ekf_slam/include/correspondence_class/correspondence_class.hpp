#ifndef CORRESPONDENCECLASS_HPP
#define CORRESPONDENCECLASS_HPP

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

#include <boost/scoped_ptr.hpp>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/inverse_chi_squared.hpp>

#include <vector>
#include <fstream>
#include <queue>
#include <math.h>
#include <algorithm>
#include <functional>
#include <list>
#include <iostream>
#include <cctype>

#include "utils_matrices/utils_matrices.hpp"

double angleLimit (double angle);

/**
 * @brief The CorrespondenceClass class
 * Auxiliar class containing all the info related to a correspondence
 * made between a landmark j and a measurement i at time t
 */
class CorrespondenceClass{

public:

    double psi_;
    double d_m_;
    Eigen::MatrixXd H_t_;
    Eigen::Matrix3d S_inverted_;
    Eigen::Vector3d landmark_pos_;
    Eigen::Vector3d nu_;
    std::pair<int, double> i_j_;

    CorrespondenceClass(const int &z_id, const double &lm_id);
    /**
     * @brief computeH
     * @param mu_hat
     * @param lm_odom
     * Computes the jacobian of the measurment model
     */

    CorrespondenceClass(CorrespondenceClass&&) = default; // forces a move constructor despite having a destructor
    CorrespondenceClass& operator=(CorrespondenceClass&&) = default; // force a move assignment anyway

    ~CorrespondenceClass();

    void computeH(const Eigen::VectorXd &mu_hat,
                  const tf::Vector3 lm_odom);
    /**
     * @brief computeS
     * @param sigma
     * @param Q
     * S = H*Q*H^T + Q
     */
    void computeMHLDistance(const Eigen::MatrixXd &sigma, const Eigen::MatrixXd &Q);
    /**
     * @brief computeNu
     * @param z_hat_i
     * @param z_i
     * Computes the innovation
     */
    void computeNu(const Eigen::Vector3d &z_hat_i, const Eigen::Vector3d &z_i);
    /**
     * @brief computeLikelihood
     * Likelihood of the correspondence mj, zi
     * It also outputs the Mahalanobis distance between z_hat_i, z_i for outlier detection
     */
    void computeLikelihood();

private:
};

#endif // CORRESPONDENCECLASS_HPP
