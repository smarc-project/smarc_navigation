#ifndef CORRESPONDENCECLASS_HPP
#define CORRESPONDENCECLASS_HPP

#include <ros/ros.h>
//#include "ekf_general/sensors_read.h"
//#include "ekf_general/plot_map.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/operation_blocked.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

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
    boost::numeric::ublas::matrix<double> H_t_;
    boost::numeric::ublas::matrix<double> S_inverted_;
    boost::numeric::ublas::vector<int> landmark_pos_;
    boost::numeric::ublas::vector<double> nu_;
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

    void computeH(const boost::numeric::ublas::vector<double> &mu_hat,
                  const tf::Vector3 lm_odom, double N_t);
    /**
     * @brief computeS
     * @param sigma
     * @param Q
     * S = H*Q*H^T + Q
     */
    void computeMHLDistance(const boost::numeric::ublas::matrix<double> &sigma, const boost::numeric::ublas::matrix<double> &Q);
    /**
     * @brief computeNu
     * @param z_hat_i
     * @param z_i
     * Computes the innovation
     */
    void computeNu(const boost::numeric::ublas::vector<double> &z_hat_i, const boost::numeric::ublas::vector<double> &z_i);
    /**
     * @brief computeLikelihood
     * Likelihood of the correspondence mj, zi
     * It also outputs the Mahalanobis distance between z_hat_i, z_i for outlier detection
     */
    void computeLikelihood();

private:
};

#endif // CORRESPONDENCECLASS_HPP
