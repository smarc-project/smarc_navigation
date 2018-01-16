#include "landmark_ml/landmark_ml.hpp"

LandmarkML::LandmarkML(const boost::numeric::ublas::vector<int> &landmark_pos){
    landmark_id_ = landmark_pos(0);
    landmark_pos_ = boost::numeric::ublas::vector<int>(3);
    landmark_pos_(0) = landmark_pos(1);
    landmark_pos_(1) = landmark_pos(2);
    landmark_pos_(2) = landmark_pos(3);
}

void LandmarkML::computeH(const boost::numeric::ublas::vector<double> &z_hat,
                          const boost::numeric::ublas::vector<double> &mu_hat,
                          const tf::StampedTransform world_base_tf){

    H_ = boost::numeric::ublas::zero_matrix<double>(3,6);
    tf::Vector3 h_jac = tf::Vector3(-1,-1,-1);
    h_jac = world_base_tf * h_jac;
    H_(0,0) = h_jac.x();
    H_(1,1) = h_jac.y();
    H_(2,2) = h_jac.z();
}

void LandmarkML::computeS(const boost::numeric::ublas::matrix<double> &sigma,
                          const boost::numeric::ublas::matrix<double> &Q){
    // Intermidiate steps
    boost::numeric::ublas::matrix<double> mat = boost::numeric::ublas::prod(H_, sigma);
    boost::numeric::ublas::matrix<double> mat1 = boost::numeric::ublas::trans(H_);
    // Computation of S
    S_ = boost::numeric::ublas::prod(mat, mat1);
    S_ += Q;
}

void LandmarkML::computeNu(const boost::numeric::ublas::vector<double> &z_hat_i,
                           const boost::numeric::ublas::vector<double> &z_i){
    nu_ = z_i - z_hat_i;
}

void LandmarkML::computeLikelihood(){
    using namespace boost::numeric::ublas;

    S_inverted_ = matrix<double> (S_.size1(), S_.size2());
    bool inverted = matrices::InvertMatrix(S_, S_inverted_);
    if(!inverted){
        ROS_ERROR("Error inverting S");
        return;
    }
    // Compute Mahalanobis distance (z_i, z_hat_j)
    vector<double> aux = prod(trans(nu_), S_inverted_);
    d_m_ = inner_prod(trans(aux), nu_);
    // Calculate the determinant on the first member of the distribution
    matrix<double> mat_aux = 2 * M_PI_2 * S_;
    double det_mat = matrices::matDeterminant(mat_aux);
    // Likelihood
    psi_ = (1 / (std::sqrt(det_mat))) * std::exp(-0.5 * d_m_);
}

double angleLimit (double angle){ // keep angle within [-pi;pi)
        return std::fmod(angle + M_PI, (M_PI * 2)) - M_PI;
}
