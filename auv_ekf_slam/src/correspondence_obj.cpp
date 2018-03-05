#include "correspondence_class/correspondence_class.hpp"


CorrespondenceClass::CorrespondenceClass(const int &z_id, const double &lm_id){
    i_j_ = std::make_pair(z_id,lm_id);
}

CorrespondenceClass::~CorrespondenceClass(){

}

void CorrespondenceClass::computeH(const boost::numeric::ublas::vector<double> &mu_hat,
                                   const tf::Vector3 lm_odom, double lm_num_t){

    using namespace std;

    // Store the landmark position
    this->landmark_pos_ = boost::numeric::ublas::vector<int>(3);
    this->landmark_pos_(0) = lm_odom.getX();
    this->landmark_pos_(1) = lm_odom.getY();
    this->landmark_pos_(2) = lm_odom.getZ();

    // Size H_t_ = 3 x (num of landmarks +1 * size_landmark * size of x_t)
    H_t_ = boost::numeric::ublas::zero_matrix<double>(3, lm_num_t*3 + 6);

    // Compute high-dimensional map of the jacobian of the measurement model
    // H_t_ has been filled in manually instead of projecting h_t_ to a higher dimension due to the higher cost of the operation
    // and the sparsity of H_t_
    H_t_(0,0) = -cos(mu_hat(4))*cos(mu_hat(5));
    H_t_(0,1) = -cos(mu_hat(4))*sin(mu_hat(5));
    H_t_(0,2) = sin(mu_hat(4));
    H_t_(0,3) = 0;
    H_t_(0,4) = mu_hat(2)*cos(mu_hat(4)) - lm_odom.getZ()*cos(mu_hat(4)) - lm_odom.getX()*cos(mu_hat(5))*sin(mu_hat(4)) - lm_odom.getY()*sin(mu_hat(4))
            *sin(mu_hat(5)) + mu_hat(0)*cos(mu_hat(5))*sin(mu_hat(4)) + mu_hat(1)*sin(mu_hat(4))*sin(mu_hat(5));
    H_t_(0,5) = cos(mu_hat(4))*(lm_odom.getY()*cos(mu_hat(5)) - lm_odom.getX()*sin(mu_hat(5)) - mu_hat(1)*cos(mu_hat(5)) + mu_hat(0)*sin(mu_hat(5)));

    H_t_(1,0) = cos(mu_hat(3))*sin(mu_hat(5)) - cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3));
    H_t_(1,1) = - cos(mu_hat(3))*cos(mu_hat(5)) - sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));
    H_t_(1,2) = -cos(mu_hat(4))*sin(mu_hat(3));
    H_t_(1,3) = lm_odom.getZ()*cos(mu_hat(4))*cos(mu_hat(3)) - mu_hat(2)*cos(mu_hat(4))*cos(mu_hat(3)) - lm_odom.getY()*cos(mu_hat(5))*sin(mu_hat(3))
            + lm_odom.getX()*sin(mu_hat(3))*sin(mu_hat(5)) + mu_hat(1)*cos(mu_hat(5))*sin(mu_hat(3)) - mu_hat(0)*sin(mu_hat(3))*sin(mu_hat(5))
            + lm_odom.getX()*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4)) + lm_odom.getY()*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5))
            - mu_hat(0)*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4)) - mu_hat(1)*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5));
    H_t_(1,4) = -sin(mu_hat(3))*(lm_odom.getZ()*sin(mu_hat(4)) - mu_hat(2)*sin(mu_hat(4)) - lm_odom.getX()*cos(mu_hat(4))*cos(mu_hat(5))
              - lm_odom.getY()*cos(mu_hat(4))*sin(mu_hat(5)) + mu_hat(0)*cos(mu_hat(4))*cos(mu_hat(5)) + mu_hat(1)*cos(mu_hat(4))*sin(mu_hat(5)));
    H_t_(1,5) = mu_hat(0)*cos(mu_hat(3))*cos(mu_hat(5)) - lm_odom.getY()*cos(mu_hat(3))*sin(mu_hat(5)) - lm_odom.getX()*cos(mu_hat(3))*cos(mu_hat(5))
            + mu_hat(1)*cos(mu_hat(3))*sin(mu_hat(5)) + lm_odom.getY()*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3))
            - lm_odom.getX()*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5)) - mu_hat(1)*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3))
            + mu_hat(0)*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));

    H_t_(2,0) = - sin(mu_hat(3))*sin(mu_hat(5)) - cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4));
    H_t_(2,1) = cos(mu_hat(5))*sin(mu_hat(3)) - cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5));
    H_t_(2,2) = -cos(mu_hat(4))*cos(mu_hat(3));
    H_t_(2,3) = lm_odom.getX()*cos(mu_hat(3))*sin(mu_hat(5)) - lm_odom.getZ()*cos(mu_hat(4))*sin(mu_hat(3)) - lm_odom.getY()*cos(mu_hat(3))*cos(mu_hat(5))
            + mu_hat(1)*cos(mu_hat(3))*cos(mu_hat(5)) + mu_hat(2)*cos(mu_hat(4))*sin(mu_hat(3)) - mu_hat(0)*cos(mu_hat(3))*sin(mu_hat(5))
            - lm_odom.getX()*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3)) - lm_odom.getY()*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5))
            + mu_hat(0)*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3)) + mu_hat(1)*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));
    H_t_(2,4) = -cos(mu_hat(3))*(lm_odom.getZ()*sin(mu_hat(4)) - mu_hat(2)*sin(mu_hat(4)) - lm_odom.getX()*cos(mu_hat(4))*cos(mu_hat(5))
              - lm_odom.getY()*cos(mu_hat(4))*sin(mu_hat(5)) + mu_hat(0)*cos(mu_hat(4))*cos(mu_hat(5)) + mu_hat(1)*cos(mu_hat(4))*sin(mu_hat(5)));
    H_t_(2,5) = lm_odom.getX()*cos(mu_hat(5))*sin(mu_hat(3)) + lm_odom.getY()*sin(mu_hat(3))*sin(mu_hat(5)) - mu_hat(0)*cos(mu_hat(5))*sin(mu_hat(3))
            - mu_hat(1)*sin(mu_hat(3))*sin(mu_hat(5)) + lm_odom.getY()*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4))
            - lm_odom.getX()*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5)) - mu_hat(1)*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4))
            + mu_hat(0)*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5));

    H_t_(0,(i_j_.second - 1) * 3 + 6) = cos(mu_hat(4))*cos(mu_hat(5));
    H_t_(0,(i_j_.second - 1) * 3 + 6 + 1) = cos(mu_hat(4))*sin(mu_hat(5));
    H_t_(0,(i_j_.second - 1) * 3 + 6 + 2) = -sin(mu_hat(4));

    H_t_(1,(i_j_.second - 1) * 3 + 6) = cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3)) - cos(mu_hat(3))*sin(mu_hat(5));
    H_t_(1,(i_j_.second - 1) * 3 + 6 + 1) = cos(mu_hat(3))*cos(mu_hat(5)) + sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));
    H_t_(1,(i_j_.second - 1) * 3 + 6 + 2) = cos(mu_hat(4))*sin(mu_hat(3));

    H_t_(2,(i_j_.second - 1) * 3 + 6) = sin(mu_hat(3))*sin(mu_hat(5)) + cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4));
    H_t_(2,(i_j_.second - 1) * 3 + 6 + 1) = cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5)) - cos(mu_hat(5))*sin(mu_hat(3));
    H_t_(2,(i_j_.second - 1) * 3 + 6 + 2) = cos(mu_hat(4))*cos(mu_hat(3));

}

void CorrespondenceClass::computeMHLDistance(const boost::numeric::ublas::matrix<double> &sigma,
                                             const boost::numeric::ublas::matrix<double> &Q){

    using namespace boost::numeric::ublas;
    matrix<double> mat = prod(H_t_, sigma);
    matrix<double> S_mat = prod(mat, trans(H_t_)) + Q;

    S_inverted_ = matrix<double> (S_mat.size1(), S_mat.size2());
    bool inverted = matrices::InvertMatrix(S_mat, S_inverted_);
    if(!inverted){
        ROS_ERROR("Error inverting S");
        return;
    }

    // Compute Mahalanobis distance (z_i, z_hat_j)
    vector<double> aux = prod(trans(nu_), S_inverted_);
    d_m_ = inner_prod(trans(aux), nu_);

}

void CorrespondenceClass::computeNu(const boost::numeric::ublas::vector<double> &z_hat_i,
                                    const boost::numeric::ublas::vector<double> &z_i){
    nu_ = z_i - z_hat_i;
}

void CorrespondenceClass::computeLikelihood(){
    using namespace boost::numeric::ublas;

//    // Calculate the determinant on the first member of the distribution
//    matrix<double> mat_aux = 2 * M_PI_2 * S_;
//    double det_mat = matrices::matDeterminant(mat_aux);

//    // Likelihood
//    psi_ = (1 / (std::sqrt(det_mat))) * std::exp(-0.5 * d_m_);
}

double angleLimit (double angle){ // keep angle within [-pi;pi)
        return std::fmod(angle + M_PI, (M_PI * 2)) - M_PI;
}
