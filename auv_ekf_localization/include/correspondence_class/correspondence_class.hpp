/* Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
    boost::numeric::ublas::matrix<double> H_;
    boost::numeric::ublas::matrix<double> S_;
    boost::numeric::ublas::matrix<double> S_inverted_;
    boost::numeric::ublas::vector<int> landmark_pos_;
    boost::numeric::ublas::vector<double> nu_;
    int landmark_id_;

    CorrespondenceClass(const double &lm_id);
    /**
     * @brief computeH
     * @param mu_hat
     * @param lm_odom
     * Computes the jacobian of the measurment model
     */
    void computeH(const boost::numeric::ublas::vector<double> &mu_hat,
                  const tf::Vector3 lm_odom);
    /**
     * @brief computeS
     * @param sigma
     * @param Q
     * S = H*Q*H^T + Q
     */
    void computeS(const boost::numeric::ublas::matrix<double> &sigma, const boost::numeric::ublas::matrix<double> &Q);
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
