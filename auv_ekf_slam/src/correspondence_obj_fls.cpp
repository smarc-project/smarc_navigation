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

#include "correspondence_class/correspondence_fls.hpp"


CorrespondenceFLS::CorrespondenceFLS(): CorrespondenceClass(){}

CorrespondenceFLS::CorrespondenceFLS(const int &z_id, const double &lm_id): CorrespondenceClass(z_id, lm_id){
    i_j_ = std::make_pair(z_id,lm_id);
}

CorrespondenceFLS::~CorrespondenceFLS(){}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> CorrespondenceFLS::measModel(const tf::Vector3& lm_j_map, const tf::Transform& tf_map_sensor){

    double scaling = 400.0/17.0;    // TODO: check this value
    tf::Vector3 z_hat_fls = tf_map_sensor * lm_j_map;   // Expected meas in fls frame and m
    Eigen::MatrixXd h_2;
    h_2.setZero(2,3);
    Eigen::Vector3d zprime(z_hat_fls.getX(), 0.0, z_hat_fls.getZ());
    h_2.row(0) = (1.0/zprime.norm()) * zprime;
    h_2(1,1) = -1;
    h_2 *= scaling;

    Eigen::Vector2d z_hat_fls_pix = h_2 * Eigen::Vector3d(z_hat_fls.getX(), z_hat_fls.getY(), z_hat_fls.getZ());
    Eigen::Vector3d z_expected = Eigen::Vector3d(z_hat_fls_pix(0), z_hat_fls_pix(1), 0.0);
    Eigen::Vector3d z_expected_sensor = Eigen::Vector3d(z_hat_fls.getX(), z_hat_fls.getY(), z_hat_fls.getZ());

    return std::make_tuple(z_expected, z_expected_sensor);
}


Eigen::VectorXd CorrespondenceFLS::backProjectNewLM(const Eigen::VectorXd& z_t, const tf::Transform& tf_map_sensor){
    tf::Vector3 new_lm_fls = tf::Vector3(z_t(0), -z_t(1), z_t(2));  // To polar coordinates to scale from pixels to meters
    double theta = std::atan2(new_lm_fls.getY(), new_lm_fls.getX());
    double rho = std::sqrt(std::pow(new_lm_fls.getX(),2) + std::pow(new_lm_fls.getY(),2));
    double rho_scaled = 17.0/400.0 * rho;

    tf::Vector3 lm_fls_real(rho_scaled * std::cos(theta),
                            rho_scaled * std::sin(theta),
                            0);

    // Transform to odom frame
    tf::Vector3 new_lm_map = tf_map_sensor *  lm_fls_real;

    return Eigen::Vector3d(new_lm_map.getX(), new_lm_map.getY(), new_lm_map.getZ());
}


void CorrespondenceFLS::computeH(const h_comp h_comps, const tf::Vector3 lm_odom, const Eigen::Vector3d z_hat_fls_m){

    // Store the landmark position
    this->landmark_pos_(0) = lm_odom.getX();
    this->landmark_pos_(1) = lm_odom.getY();
    this->landmark_pos_(2) = lm_odom.getZ();

    // Size H_t_ = 3 x (num of landmarks +1 * size_landmark * size of x_t)
    Eigen::MatrixXd h_t;
    h_t.setZero(3,3);

    // Compute high-dimensional map of the jacobian of the measurement model
    // H_t_ has been filled in manually instead of projecting h_t_ to a higher dimension due to the higher cost of the operation
    // and the sparsity of H_t_
    Eigen::MatrixXd h_1, h_2;

    // First component of the jacobian
    h_2.setZero(2,3);
    Eigen::Vector3d zprime(z_hat_fls_m(0), 0.0, z_hat_fls_m(2));
    h_2.row(0) = (1.0/zprime.norm()) * zprime;
    h_2(1,1) = -1;
    h_2 *= 400.0/17.0;

    // Map h_t_ to the correct dimension
    h_1.setZero(3,9);

    using namespace std;
    h_1(0,0) = -h_comps.c_4*h_comps.c_5;
    h_1(0,1) = -h_comps.c_4*h_comps.s_5;
    h_1(0,2) = h_comps.s_4;
    h_1(0,3) = 0;
    h_1(0,4) = h_comps.mu_2*h_comps.c_4 - lm_odom.getZ()*h_comps.c_4 - lm_odom.getX()*h_comps.c_5*h_comps.s_4 - lm_odom.getY()*h_comps.s_4
            *h_comps.s_5 + h_comps.mu_0*h_comps.c_5*h_comps.s_4 + h_comps.mu_1*h_comps.s_4*h_comps.s_5;
    h_1(0,5) = h_comps.c_4*(lm_odom.getY()*h_comps.c_5 - lm_odom.getX()*h_comps.s_5 - h_comps.mu_1*h_comps.c_5 + h_comps.mu_0*h_comps.s_5);
    h_1(0,6) = h_comps.c_4*h_comps.c_5;
    h_1(0,7) = h_comps.c_4*h_comps.s_5;
    h_1(0,8) = -h_comps.s_4;

    h_1(1,0) = h_comps.c_3*h_comps.s_5 - h_comps.c_5*h_comps.s_4*h_comps.s_3;
    h_1(1,1) = - h_comps.c_3*h_comps.c_5 - h_comps.s_4*h_comps.s_3*h_comps.s_5;
    h_1(1,2) = -h_comps.c_4*h_comps.s_3;
    h_1(1,3) = lm_odom.getZ()*h_comps.c_4*h_comps.c_3 - h_comps.mu_2*h_comps.c_4*h_comps.c_3 - lm_odom.getY()*h_comps.c_5*h_comps.s_3
            + lm_odom.getX()*h_comps.s_3*h_comps.s_5 + h_comps.mu_1*h_comps.c_5*h_comps.s_3 - h_comps.mu_0*h_comps.s_3*h_comps.s_5
            + lm_odom.getX()*h_comps.c_3*h_comps.c_5*h_comps.s_4 + lm_odom.getY()*h_comps.c_3*h_comps.s_4*h_comps.s_5
            - h_comps.mu_0*h_comps.c_3*h_comps.c_5*h_comps.s_4 - h_comps.mu_1*h_comps.c_3*h_comps.s_4*h_comps.s_5;
    h_1(1,4) = -h_comps.s_3*(lm_odom.getZ()*h_comps.s_4 - h_comps.mu_2*h_comps.s_4 - lm_odom.getX()*h_comps.c_4*h_comps.c_5
              - lm_odom.getY()*h_comps.c_4*h_comps.s_5 + h_comps.mu_0*h_comps.c_4*h_comps.c_5 + h_comps.mu_1*h_comps.c_4*h_comps.s_5);
    h_1(1,5) = h_comps.mu_0*h_comps.c_3*h_comps.c_5 - lm_odom.getY()*h_comps.c_3*h_comps.s_5 - lm_odom.getX()*h_comps.c_3*h_comps.c_5
            + h_comps.mu_1*h_comps.c_3*h_comps.s_5 + lm_odom.getY()*h_comps.c_5*h_comps.s_4*h_comps.s_3
            - lm_odom.getX()*h_comps.s_4*h_comps.s_3*h_comps.s_5 - h_comps.mu_1*h_comps.c_5*h_comps.s_4*h_comps.s_3
            + h_comps.mu_0*h_comps.s_4*h_comps.s_3*h_comps.s_5;
    h_1(1,6) = h_comps.c_5*h_comps.s_4*h_comps.s_3 - h_comps.c_3*h_comps.s_5;
    h_1(1,7) = h_comps.c_3*h_comps.c_5 + h_comps.s_4*h_comps.s_3*h_comps.s_5;
    h_1(1,8) = h_comps.c_4*h_comps.s_3;

    h_1(2,0) = - h_comps.s_3*h_comps.s_5 - h_comps.c_3*h_comps.c_5*h_comps.s_4;
    h_1(2,1) = h_comps.c_5*h_comps.s_3 - h_comps.c_3*h_comps.s_4*h_comps.s_5;
    h_1(2,2) = -h_comps.c_4*h_comps.c_3;
    h_1(2,3) = lm_odom.getX()*h_comps.c_3*h_comps.s_5 - lm_odom.getZ()*h_comps.c_4*h_comps.s_3 - lm_odom.getY()*h_comps.c_3*h_comps.c_5
            + h_comps.mu_1*h_comps.c_3*h_comps.c_5 + h_comps.mu_2*h_comps.c_4*h_comps.s_3 - h_comps.mu_0*h_comps.c_3*h_comps.s_5
            - lm_odom.getX()*h_comps.c_5*h_comps.s_4*h_comps.s_3 - lm_odom.getY()*h_comps.s_4*h_comps.s_3*h_comps.s_5
            + h_comps.mu_0*h_comps.c_5*h_comps.s_4*h_comps.s_3 + h_comps.mu_1*h_comps.s_4*h_comps.s_3*h_comps.s_5;
    h_1(2,4) = -h_comps.c_3*(lm_odom.getZ()*h_comps.s_4 - h_comps.mu_2*h_comps.s_4 - lm_odom.getX()*h_comps.c_4*h_comps.c_5
               -lm_odom.getY()*h_comps.c_4*h_comps.s_5 + h_comps.mu_0*h_comps.c_4*h_comps.c_5 + h_comps.mu_1*h_comps.c_4*h_comps.s_5);
    h_1(2,5) = lm_odom.getX()*(h_comps.c_5*h_comps.s_3 - h_comps.c_3*h_comps.s_4*h_comps.s_5) + lm_odom.getY()*(h_comps.s_3*h_comps.s_5 + h_comps.c_3*h_comps.c_5*h_comps.s_4)
               - h_comps.mu_0*(h_comps.c_5*h_comps.s_3 - h_comps.c_3*h_comps.s_4*h_comps.s_5) - h_comps.mu_1*(h_comps.s_3*h_comps.s_5 + h_comps.c_3*h_comps.c_5*h_comps.s_4);

    h_1(2,6) = h_comps.s_3*h_comps.s_5 + h_comps.c_3*h_comps.c_5*h_comps.s_4;
    h_1(2,7) = h_comps.c_3*h_comps.s_4*h_comps.s_5 - h_comps.c_5*h_comps.s_3;
    h_1(2,8) = h_comps.c_4*h_comps.c_3;

    h_1 = h_comps.R_fls_base_ * h_1;

    H_t_ = h_2 * h_1;
}

void CorrespondenceFLS::computeMHLDistance(const Eigen::MatrixXd &sigma, const Eigen::MatrixXd &Q){

    Eigen::MatrixXd S_mat = H_t_ * sigma * H_t_.transpose() + Q;

    // TODO: check if matrix is invertible!
    S_inverted_ = S_mat.inverse();
    d_m_ = nu_.transpose() * S_inverted_ * nu_;
}

void CorrespondenceFLS::computeNu(const Eigen::Vector3d &z_hat_i, const Eigen::Vector3d &z_i){
    nu_ = Eigen::Vector2d();
    nu_(0) = z_i(0) - z_hat_i(0); // nu in pixels
    nu_(1) = z_i(1) - z_hat_i(1);
}

void CorrespondenceFLS::computeLikelihood(){

    //    // Calculate the determinant on the first member of the distribution
    //    matrix<double> mat_aux = 2 * M_PI_2 * S_;
    //    double det_mat = matrices::matDeterminant(mat_aux);

    //    // Likelihood
    //    psi_ = (1 / (std::sqrt(det_mat))) * std::exp(-0.5 * d_m_);
}




