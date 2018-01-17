#include "landmark_ml/landmark_ml.hpp"

LandmarkML::LandmarkML(const boost::numeric::ublas::vector<int> &landmark_pos){
    landmark_id_ = landmark_pos(0);
    landmark_pos_ = boost::numeric::ublas::vector<int>(3);
    landmark_pos_(0) = landmark_pos(1);
    landmark_pos_(1) = landmark_pos(2);
    landmark_pos_(2) = landmark_pos(3);
}

void LandmarkML::computeH(const boost::numeric::ublas::vector<double> &mu_hat,
                          const tf::Vector3 lm_odom){

    H_ = boost::numeric::ublas::identity_matrix<double>(3,6);

    using namespace std;
    H_(0,3) = lm_odom.getY()*(sin(mu_hat(3))*sin(mu_hat(5)) + cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4))) + lm_odom.getZ()*cos(mu_hat(3))*sin(mu_hat(5));
    H_(0,4) = cos(mu_hat(5))*(lm_odom.getZ()*cos(mu_hat(4))*cos(mu_hat(5)) - lm_odom.getX()*sin(mu_hat(4)) + lm_odom.getY()*cos(mu_hat(4))*sin(mu_hat(3)));
    H_(0,5) = lm_odom.getZ()*(cos(mu_hat(5))*sin(mu_hat(3)) - 2*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(5))) - lm_odom.getY()*(cos(mu_hat(3))*cos(mu_hat(5))
              + sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5))) - lm_odom.getX()*cos(mu_hat(4))*sin(mu_hat(5));

    H_(1,3) = lm_odom.getY()*(cos(mu_hat(5))*sin(mu_hat(3)) + cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5))) + lm_odom.getZ()*cos(mu_hat(3))*cos(mu_hat(5));
    H_(1,4) = sin(mu_hat(5))*(lm_odom.getZ()*cos(mu_hat(4))*cos(mu_hat(5)) - lm_odom.getX()*sin(mu_hat(4)) + lm_odom.getY()*cos(mu_hat(4))*sin(mu_hat(3)));
    H_(1,5) = lm_odom.getY()*(cos(mu_hat(3))*sin(mu_hat(5)) + cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3))) - lm_odom.getZ()*(sin(mu_hat(3))*sin(mu_hat(5))
              - pow(cos(mu_hat(5)),2)*sin(mu_hat(4)) + sin(mu_hat(4))*pow(sin(mu_hat(5)),2)) + lm_odom.getX()*cos(mu_hat(4))*cos(mu_hat(5));

    H_(2,3) = cos(mu_hat(4))*(lm_odom.getY()*cos(mu_hat(3)) - lm_odom.getZ()*sin(mu_hat(3)));
    H_(2,4) = -1 * lm_odom.getX()*cos(mu_hat(4)) - lm_odom.getZ()*cos(mu_hat(3))*sin(mu_hat(4)) - lm_odom.getY()*sin(mu_hat(4))*sin(mu_hat(3));
    H_(2,5) = 0;
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
    nu_(0) = 0; // Remove innovation in x for testing
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
