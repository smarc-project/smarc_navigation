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

#ifndef EKF_SLAM_CORE_HPP
#define EKF_SLAM_CORE_HPP

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <Eigen/SparseCore>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include "correspondence_class/correspondence_class.hpp"
#include "correspondence_class/correspondence_mbes.hpp"
#include "correspondence_class/correspondence_fls.hpp"

#include <queue>
#include <math.h>

#include <boost/thread/mutex.hpp>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/inverse_chi_squared.hpp>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

#include "utils/ekf_utils.hpp"
#include "munkres/munkres.h"

class EKFCore{

public:

    EKFCore(Eigen::VectorXd& mu, Eigen::MatrixXd& Sigma, Eigen::MatrixXd& R, Eigen::MatrixXd& Q_fls, Eigen::MatrixXd &Q_mbes,
            double& lambda_fls, double& lambda_mbes, tf::StampedTransform& tf_base_sensor, const double mh_dist_fls, const double mh_dist_mbes);
    ~EKFCore();
    std::tuple<Eigen::VectorXd, Eigen::MatrixXd> ekfUpdate();
    void predictMotion(nav_msgs::Odometry odom_reading);
    void batchDataAssociation(std::vector<Eigen::Vector3d> z_t, const utils::MeasSensor& sens_type);

private:

    // System state variables
    Eigen::VectorXd mu_;
    Eigen::MatrixXd Sigma_;
    Eigen::VectorXd mu_hat_;
    Eigen::MatrixXd Sigma_hat_;
    Eigen::Vector3d mu_auv_odom_;

    // Noise models
    Eigen::MatrixXd R_;
    Eigen::MatrixXd Q_fls_;
    Eigen::MatrixXd Q_mbes_;

    // Mapping variables
    tf::StampedTransform tf_base_sensor_;
    tf::Transform tf_sensor_base_;
    double lambda_mbes_;
    double lambda_fls_;
    double mh_dist_fls_;
    double mh_dist_mbes_;
    int lm_num_;

    void predictBatchMeasurement(const Eigen::Vector3d &landmark_j,
                                const Eigen::Vector3d &z_i,
                                unsigned int i,
                                unsigned int j,
                                const tf::Transform &transf_base_odom,
                                const Eigen::MatrixXd &temp_sigma,
                                h_comp h_comps,
                                const utils::MeasSensor &sens_type,
                                std::vector<CorrespondenceClass> &corresp_i_list,
                                Eigen::MatrixXd &corresp_table);

    void sequentialUpdate(const CorrespondenceClass &c_i_j, Eigen::MatrixXd temp_sigma);

};

#endif // EKF_SLAM_CORE_HPP
