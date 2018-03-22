#include "correspondence_class/correspondence_class.hpp"


CorrespondenceClass::CorrespondenceClass(const int &z_id, const double &lm_id){
    i_j_ = std::make_pair(z_id,lm_id);
}

CorrespondenceClass::~CorrespondenceClass(){

}

void CorrespondenceClass::computeH(const h_comp h_comps,
                                   const tf::Vector3 lm_odom,
                                   const Eigen::Vector3d z_hat_fls_m){

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
    h_2(0,0) = 1;
    Eigen::Vector3d zprime(0.0, z_hat_fls_m(1), z_hat_fls_m(2));
    h_2.row(1) = (1.0/zprime.norm()) * zprime;
    h_2 *= 400.0/17.0;

    // Map h_t_ to the correct dimension
    h_1.setZero(3,9);
    h_1.block(0,0,3,3) = -1 * Eigen::MatrixXd::Identity(3,3);

    using namespace std;    
    h_1(0,0) = -h_comps.c_4*h_comps.c_5;
    h_1(0,1) = -h_comps.c_4*h_comps.s_5;
    h_1(0,2) = h_comps.s_4;
    h_1(0,3) = 0;
    h_1(0,4) = h_comps.mu_2*h_comps.c_4 - lm_odom.getZ()*h_comps.c_4 - lm_odom.getX()*h_comps.c_5*h_comps.s_4 - lm_odom.getY()*h_comps.s_4
            *h_comps.s_5 + h_comps.mu_0*h_comps.c_5*h_comps.s_4 + h_comps.mu_1*h_comps.s_4*h_comps.s_5;
    h_1(0,5) = h_comps.c_4*(lm_odom.getY()*h_comps.c_5 - lm_odom.getX()*h_comps.s_5 - h_comps.mu_1*h_comps.c_5 + h_comps.mu_0*h_comps.s_5);

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

    h_1(2,0) = - h_comps.s_3*h_comps.s_5 - h_comps.c_3*h_comps.c_5*h_comps.s_4;
    h_1(2,1) = h_comps.c_5*h_comps.s_3 - h_comps.c_3*h_comps.s_4*h_comps.s_5;
    h_1(2,2) = -h_comps.c_4*h_comps.c_3;
    h_1(2,3) = lm_odom.getX()*h_comps.c_3*h_comps.s_5 - lm_odom.getZ()*h_comps.c_4*h_comps.s_3 - lm_odom.getY()*h_comps.c_3*h_comps.c_5
            + h_comps.mu_1*h_comps.c_3*h_comps.c_5 + h_comps.mu_2*h_comps.c_4*h_comps.s_3 - h_comps.mu_0*h_comps.c_3*h_comps.s_5
            - lm_odom.getX()*h_comps.c_5*h_comps.s_4*h_comps.s_3 - lm_odom.getY()*h_comps.s_4*h_comps.s_3*h_comps.s_5
            + h_comps.mu_0*h_comps.c_5*h_comps.s_4*h_comps.s_3 + h_comps.mu_1*h_comps.s_4*h_comps.s_3*h_comps.s_5;
    h_1(2,4) = -h_comps.c_3*(lm_odom.getZ()*h_comps.s_4 - h_comps.mu_2*h_comps.s_4 - lm_odom.getX()*h_comps.c_4*h_comps.c_5
              - lm_odom.getY()*h_comps.c_4*h_comps.s_5 + h_comps.mu_0*h_comps.c_4*h_comps.c_5 + h_comps.mu_1*h_comps.c_4*h_comps.s_5);
    h_1(2,5) = lm_odom.getX()*h_comps.c_5*h_comps.s_3 + lm_odom.getY()*h_comps.s_3*h_comps.s_5 - h_comps.mu_0*h_comps.c_5*h_comps.s_3
            - h_comps.mu_1*h_comps.s_3*h_comps.s_5 + lm_odom.getY()*h_comps.c_3*h_comps.c_5*h_comps.s_4
            - lm_odom.getX()*h_comps.c_3*h_comps.s_4*h_comps.s_5 - h_comps.mu_1*h_comps.c_3*h_comps.c_5*h_comps.s_4
            + h_comps.mu_0*h_comps.c_3*h_comps.s_4*h_comps.s_5;

    h_1(0,6) = h_comps.c_4*h_comps.c_5;
    h_1(0,7) = h_comps.c_4*h_comps.s_5;
    h_1(0,8) = -h_comps.s_4;

    h_1(1,6) = h_comps.c_5*h_comps.s_4*h_comps.s_3 - h_comps.c_3*h_comps.s_5;
    h_1(1,7) = h_comps.c_3*h_comps.c_5 + h_comps.s_4*h_comps.s_3*h_comps.s_5;
    h_1(1,8) = h_comps.c_4*h_comps.s_3;

    h_1(2,6) = h_comps.s_3*h_comps.s_5 + h_comps.c_3*h_comps.c_5*h_comps.s_4;
    h_1(2,7) = h_comps.c_3*h_comps.s_4*h_comps.s_5 - h_comps.c_5*h_comps.s_3;
    h_1(2,8) = h_comps.c_4*h_comps.c_3;


    h_1 = h_comps.R_fls_base_ * h_1;

    H_t_ = h_2 * h_1;
}

void CorrespondenceClass::computeMHLDistance(const Eigen::MatrixXd &sigma,
                                             const Eigen::MatrixXd &Q){

    Eigen::MatrixXd S_mat = H_t_ * sigma * H_t_.transpose() + Q;

    // TODO: check if matrix is invertible!
    S_inverted_ = S_mat.inverse();
    d_m_ = nu_.transpose() * S_inverted_ * nu_;
}

void CorrespondenceClass::computeNu(const Eigen::Vector3d &z_hat_i, const Eigen::Vector3d &z_i){
    nu_(0) = z_i(0) - z_hat_i(0); // nu in pixels
    nu_(1) = z_i(1) - z_hat_i(1);
}

void CorrespondenceClass::computeLikelihood(){

//    // Calculate the determinant on the first member of the distribution
//    matrix<double> mat_aux = 2 * M_PI_2 * S_;
//    double det_mat = matrices::matDeterminant(mat_aux);

//    // Likelihood
//    psi_ = (1 / (std::sqrt(det_mat))) * std::exp(-0.5 * d_m_);
}

double angleLimit (double angle){ // keep angle within [-pi;pi)
        return std::fmod(angle + M_PI, (M_PI * 2)) - M_PI;
}
