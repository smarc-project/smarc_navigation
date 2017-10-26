#include "landmark_ml.hpp"


LandmarkML::LandmarkML(unsigned int &landmark_id, const boost::numeric::ublas::vector<int> &landmark_pos):
    landmark_id_(landmark_id){
    landmark_pos_ = boost::numeric::ublas::vector<int>(2);
    landmark_pos_.assign(landmark_pos);
}

void LandmarkML::computeH(const boost::numeric::ublas::vector<double> &z_hat, const boost::numeric::ublas::vector<double> &mu_hat){
    H_ = boost::numeric::ublas::matrix<double>(2,3);
    H_(0,0) = (mu_hat(0) - landmark_pos_(0))/z_hat(0);
    H_(1,0) = (-1 * mu_hat(1) - landmark_pos_(1))/std::sqrt(z_hat(0));
    H_(0,1) = (mu_hat(1) - landmark_pos_(1))/z_hat(0);
    H_(1,1) = (mu_hat(0) - landmark_pos_(0))/std::sqrt(z_hat(0));
    H_(0,2) = 0;
    H_(1,2) = -1;
}

void LandmarkML::computeS(const boost::numeric::ublas::matrix<double> &sigma, const boost::numeric::ublas::matrix<double> &Q){
    // Intermidiate steps
    boost::numeric::ublas::matrix<double> mat = boost::numeric::ublas::prod(H_, sigma);
    boost::numeric::ublas::matrix<double> mat1 = boost::numeric::ublas::trans(H_);
    // Computation of S
    S_ = boost::numeric::ublas::prod(mat, mat1);
    S_ += Q;
}

void LandmarkML::computeNu(const boost::numeric::ublas::vector<double> &z_hat_i, const boost::numeric::ublas::vector<double> &z_i){
    nu_ = z_i - z_hat_i;
}

// TODO_NACHO: NORMALIZE Gaussian pdf!!!!!!
void LandmarkML::computeLikelihood(){
    S_inverted_ = boost::numeric::ublas::matrix<double> (S_.size1(), S_.size2());
    bool inverted = matrices::InvertMatrix(S_, S_inverted_);
    if(!inverted){
        std::cout << "Error inverting S" << std::endl;
        return;
    }
    // Compute Mahalanobis distance (z_i, z_hat_j)
    boost::numeric::ublas::vector<double> aux = boost::numeric::ublas::prod(boost::numeric::ublas::trans(nu_), S_inverted_);
    d_m_ = boost::numeric::ublas::inner_prod(boost::numeric::ublas::trans(aux), nu_);
    // Calculate the determinant on the first member of the distribution
    boost::numeric::ublas::matrix<double> mat_aux = M_PI_2 * S_;
    double det_mat = matrices::matDeterminant(mat_aux);
    // Likelihood
    psi_ = (1 / (std::sqrt(det_mat))) * std::exp(-0.5 * d_m_);
}

