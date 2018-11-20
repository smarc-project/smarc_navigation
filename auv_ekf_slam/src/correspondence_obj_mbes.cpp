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

#include "correspondence_class/correspondence_mbes.hpp"


CorrespondenceMBES::CorrespondenceMBES(): CorrespondenceClass(){}


CorrespondenceMBES::CorrespondenceMBES(const int &z_id, const double &lm_id): CorrespondenceClass(z_id, lm_id){
    i_j_ = std::make_pair(z_id,lm_id);
}

CorrespondenceMBES::~CorrespondenceMBES(){}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> CorrespondenceMBES::measModel(const tf::Vector3& lm_j_map, const tf::Transform& tf_sensor_map){
    tf::Vector3 z_hat_base = tf_sensor_map * lm_j_map;
    Eigen::Vector3d z_expected = Eigen::Vector3d(z_hat_base.getX(),
                                                 z_hat_base.getY(),
                                                 z_hat_base.getZ());

    Eigen::Vector3d z_expected_sensor = Eigen::Vector3d();

    return std::make_tuple(z_expected, z_expected_sensor);
}



Eigen::VectorXd CorrespondenceMBES::backProjectNewLM(const Eigen::VectorXd &z_t, const tf::Transform &tf_map_sensor){
    // New LM as a transform from base to map frame
    tf::Vector3 new_lm_map = tf_map_sensor * tf::Vector3(z_t(0), z_t(1), z_t(2));

    return Eigen::Vector3d(new_lm_map.getX(), new_lm_map.getY(), new_lm_map.getZ());
}


void CorrespondenceMBES::computeH(const h_comp h_comps, const tf::Vector3 lm_odom, const Eigen::Vector3d){
    using namespace std;

    // Store the landmark position
    this->landmark_pos_(0) = lm_odom.getX();
    this->landmark_pos_(1) = lm_odom.getY();
    this->landmark_pos_(2) = lm_odom.getZ();

    // Size H_t_ = 3 x (num of landmarks +1 * size_landmark * size of x_t)
    H_t_ = Eigen::MatrixXd(3, 9);

    // Compute high-dimensional map of the jacobian of the measurement model
    // H_t_ has been filled in manually instead of projecting h_t_ to a higher dimension due to the higher cost of the operation
    // and the sparsity of H_t_
    H_t_(0,0) = -h_comps.c_4*h_comps.c_5;
    H_t_(0,1) = -h_comps.c_4*h_comps.s_5;
    H_t_(0,2) = h_comps.s_4;
    H_t_(0,3) = 0;
    H_t_(0,4) = h_comps.mu_2*h_comps.c_4 - lm_odom.getZ()*h_comps.c_4 - lm_odom.getX()*h_comps.c_5*h_comps.s_4 - lm_odom.getY()*h_comps.s_4
            *h_comps.s_5 + h_comps.mu_0*h_comps.c_5*h_comps.s_4 + h_comps.mu_1*h_comps.s_4*h_comps.s_5;
    H_t_(0,5) = h_comps.c_4*(lm_odom.getY()*h_comps.c_5 - lm_odom.getX()*h_comps.s_5 - h_comps.mu_1*h_comps.c_5 + h_comps.mu_0*h_comps.s_5);

    H_t_(1,0) = h_comps.c_3*h_comps.s_5 - h_comps.c_5*h_comps.s_4*h_comps.s_3;
    H_t_(1,1) = - h_comps.c_3*h_comps.c_5 - h_comps.s_4*h_comps.s_3*h_comps.s_5;
    H_t_(1,2) = -h_comps.c_4*h_comps.s_3;
    H_t_(1,3) = lm_odom.getZ()*h_comps.c_4*h_comps.c_3 - h_comps.mu_2*h_comps.c_4*h_comps.c_3 - lm_odom.getY()*h_comps.c_5*h_comps.s_3
            + lm_odom.getX()*h_comps.s_3*h_comps.s_5 + h_comps.mu_1*h_comps.c_5*h_comps.s_3 - h_comps.mu_0*h_comps.s_3*h_comps.s_5
            + lm_odom.getX()*h_comps.c_3*h_comps.c_5*h_comps.s_4 + lm_odom.getY()*h_comps.c_3*h_comps.s_4*h_comps.s_5
            - h_comps.mu_0*h_comps.c_3*h_comps.c_5*h_comps.s_4 - h_comps.mu_1*h_comps.c_3*h_comps.s_4*h_comps.s_5;
    H_t_(1,4) = -h_comps.s_3*(lm_odom.getZ()*h_comps.s_4 - h_comps.mu_2*h_comps.s_4 - lm_odom.getX()*h_comps.c_4*h_comps.c_5
              - lm_odom.getY()*h_comps.c_4*h_comps.s_5 + h_comps.mu_0*h_comps.c_4*h_comps.c_5 + h_comps.mu_1*h_comps.c_4*h_comps.s_5);
    H_t_(1,5) = h_comps.mu_0*h_comps.c_3*h_comps.c_5 - lm_odom.getY()*h_comps.c_3*h_comps.s_5 - lm_odom.getX()*h_comps.c_3*h_comps.c_5
            + h_comps.mu_1*h_comps.c_3*h_comps.s_5 + lm_odom.getY()*h_comps.c_5*h_comps.s_4*h_comps.s_3
            - lm_odom.getX()*h_comps.s_4*h_comps.s_3*h_comps.s_5 - h_comps.mu_1*h_comps.c_5*h_comps.s_4*h_comps.s_3
            + h_comps.mu_0*h_comps.s_4*h_comps.s_3*h_comps.s_5;

    H_t_(2,0) = - h_comps.s_3*h_comps.s_5 - h_comps.c_3*h_comps.c_5*h_comps.s_4;
    H_t_(2,1) = h_comps.c_5*h_comps.s_3 - h_comps.c_3*h_comps.s_4*h_comps.s_5;
    H_t_(2,2) = -h_comps.c_4*h_comps.c_3;
    H_t_(2,3) = lm_odom.getX()*h_comps.c_3*h_comps.s_5 - lm_odom.getZ()*h_comps.c_4*h_comps.s_3 - lm_odom.getY()*h_comps.c_3*h_comps.c_5
            + h_comps.mu_1*h_comps.c_3*h_comps.c_5 + h_comps.mu_2*h_comps.c_4*h_comps.s_3 - h_comps.mu_0*h_comps.c_3*h_comps.s_5
            - lm_odom.getX()*h_comps.c_5*h_comps.s_4*h_comps.s_3 - lm_odom.getY()*h_comps.s_4*h_comps.s_3*h_comps.s_5
            + h_comps.mu_0*h_comps.c_5*h_comps.s_4*h_comps.s_3 + h_comps.mu_1*h_comps.s_4*h_comps.s_3*h_comps.s_5;
    H_t_(2,4) = -h_comps.c_3*(lm_odom.getZ()*h_comps.s_4 - h_comps.mu_2*h_comps.s_4 - lm_odom.getX()*h_comps.c_4*h_comps.c_5
              - lm_odom.getY()*h_comps.c_4*h_comps.s_5 + h_comps.mu_0*h_comps.c_4*h_comps.c_5 + h_comps.mu_1*h_comps.c_4*h_comps.s_5);
    H_t_(2,5) = lm_odom.getX()*h_comps.c_5*h_comps.s_3 + lm_odom.getY()*h_comps.s_3*h_comps.s_5 - h_comps.mu_0*h_comps.c_5*h_comps.s_3
            - h_comps.mu_1*h_comps.s_3*h_comps.s_5 + lm_odom.getY()*h_comps.c_3*h_comps.c_5*h_comps.s_4
            - lm_odom.getX()*h_comps.c_3*h_comps.s_4*h_comps.s_5 - h_comps.mu_1*h_comps.c_3*h_comps.c_5*h_comps.s_4
            + h_comps.mu_0*h_comps.c_3*h_comps.s_4*h_comps.s_5;

    H_t_(0,6) = h_comps.c_4*h_comps.c_5;
    H_t_(0,7) = h_comps.c_4*h_comps.s_5;
    H_t_(0,8) = -h_comps.s_4;

    H_t_(1,6) = h_comps.c_5*h_comps.s_4*h_comps.s_3 - h_comps.c_3*h_comps.s_5;
    H_t_(1,7) = h_comps.c_3*h_comps.c_5 + h_comps.s_4*h_comps.s_3*h_comps.s_5;
    H_t_(1,8) = h_comps.c_4*h_comps.s_3;

    H_t_(2,6) = h_comps.s_3*h_comps.s_5 + h_comps.c_3*h_comps.c_5*h_comps.s_4;
    H_t_(2,7) = h_comps.c_3*h_comps.s_4*h_comps.s_5 - h_comps.c_5*h_comps.s_3;
    H_t_(2,8) = h_comps.c_4*h_comps.c_3;
}

void CorrespondenceMBES::computeMHLDistance(const Eigen::MatrixXd &sigma, const Eigen::MatrixXd &Q){
    Eigen::Matrix3d S_mat = H_t_ * sigma * H_t_.transpose() + Q;

    // TODO: check if matrix is invertible!
    S_inverted_ = S_mat.inverse();
    d_m_ = nu_.transpose() * S_inverted_ * nu_;
}

void CorrespondenceMBES::computeNu(const Eigen::Vector3d &z_hat_i, const Eigen::Vector3d &z_i){
        nu_ = z_i - z_hat_i;
}

void CorrespondenceMBES::computeLikelihood(){
    //    // Calculate the determinant on the first member of the distribution
    //    matrix<double> mat_aux = 2 * M_PI_2 * S_;
    //    double det_mat = matrices::matDeterminant(mat_aux);

    //    // Likelihood
    //    psi_ = (1 / (std::sqrt(det_mat))) * std::exp(-0.5 * d_m_);
}


