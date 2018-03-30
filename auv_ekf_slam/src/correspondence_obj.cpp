#include "correspondence_class/correspondence_class.hpp"


CorrespondenceClass::CorrespondenceClass(const int &z_id, const double &lm_id){
    i_j_ = std::make_pair(z_id,lm_id);
}

CorrespondenceClass::~CorrespondenceClass(){

}

void CorrespondenceClass::computeH(const h_comp h_comps,
                                   const tf::Vector3 lm_odom,
                                   const Eigen::Vector3d z_hat_fls_m){

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

void CorrespondenceClass::computeMHLDistance(const Eigen::MatrixXd &sigma,
                                             const Eigen::MatrixXd &Q){

    Eigen::Matrix3d S_mat = H_t_ * sigma * H_t_.transpose() + Q;

    // TODO: check if matrix is invertible!
    S_inverted_ = S_mat.inverse();
//    if(!inverted){
//        ROS_ERROR("Error inverting S");
//        return;
//    }

    // Compute Mahalanobis distance (z_i, z_hat_j)
    d_m_ = nu_.transpose() * S_inverted_ * nu_;
}

void CorrespondenceClass::computeNu(const Eigen::Vector3d &z_hat_i, const Eigen::Vector3d &z_i){
    nu_ = z_i - z_hat_i;
}

void CorrespondenceClass::computeLikelihood(){

//    // Calculate the determinant on the first member of the distribution
//    matrix<double> mat_aux = 2 * M_PI_2 * S_;
//    double det_mat = matrices::matDeterminant(mat_aux);

//    // Likelihood
//    psi_ = (1 / (std::sqrt(det_mat))) * std::exp(-0.5 * d_m_);
}

