#include "correspondence_class/correspondence_class.hpp"


CorrespondenceClass::CorrespondenceClass(const int &z_id, const double &lm_id){
    i_j_ = std::make_pair(z_id,lm_id);
}

CorrespondenceClass::~CorrespondenceClass(){

}

void CorrespondenceClass::computeH(const h_comp h_comps,
                                   const tf::Vector3 lm_odom){

    using namespace std;

    Eigen::SparseMatrix<double> F_x(3, 9);
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(6);
    unsigned int i = 0;
    int input = 1;
    for(unsigned int j=6; j<9; j++){
        tripletList.push_back(Eigen::Triplet<double>(i,j,input));
        i += 1;
    }
    F_x.setFromTriplets(tripletList.begin(), tripletList.end());

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
    h_t(0,0) = 1;
    Eigen::Vector3d zprime(0.0, lm_odom.getY(), lm_odom.getZ());
    h_t.row(1) = (1.0/zprime.norm()) * zprime;
    h_t(2,2) = 1;

    // Map h_t_ to the correct dimension
    this->H_t_ = h_t * F_x;
    H_t_ *= 400.0/17.0;
}

void CorrespondenceClass::computeMHLDistance(const Eigen::MatrixXd &sigma,
                                             const Eigen::MatrixXd &Q){

    Eigen::MatrixXd S_mat = H_t_ * sigma * H_t_.transpose() + Q;

    // TODO: check if matrix is invertible!
    S_inverted_ = S_mat.inverse();
    d_m_ = nu_.transpose() * S_inverted_ * nu_;
}

void CorrespondenceClass::computeNu(const Eigen::Vector3d &z_hat_i, const Eigen::Vector3d &z_i){
    nu_ = z_i - z_hat_i; // nu in metters
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
