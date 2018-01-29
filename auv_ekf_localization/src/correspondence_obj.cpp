#include "correspondence_class/correspondence_class.hpp"


CorrespondenceClass::CorrespondenceClass(const boost::numeric::ublas::vector<int> &landmark_pos){
    landmark_id_ = landmark_pos(0);
    landmark_pos_ = boost::numeric::ublas::vector<int>(3);
    landmark_pos_(0) = landmark_pos(1);
    landmark_pos_(1) = landmark_pos(2);
    landmark_pos_(2) = landmark_pos(3);
}

void CorrespondenceClass::computeH(const boost::numeric::ublas::vector<double> &mu_hat,
                          const tf::Vector3 lm_odom){

    using namespace std;
    H_ = boost::numeric::ublas::identity_matrix<double>(2,6);

    H_(0,0) = cos(mu_hat(3))*sin(mu_hat(5)) - cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3));
    H_(0,1) = - cos(mu_hat(3))*cos(mu_hat(5)) - sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));
    H_(0,2) = -cos(mu_hat(4))*sin(mu_hat(3));
    H_(0,3) = lm_odom.getZ()*cos(mu_hat(4))*cos(mu_hat(3)) - mu_hat(2)*cos(mu_hat(4))*cos(mu_hat(3)) - lm_odom.getY()*cos(mu_hat(5))*sin(mu_hat(3))
            + lm_odom.getX()*sin(mu_hat(3))*sin(mu_hat(5)) + mu_hat(1)*cos(mu_hat(5))*sin(mu_hat(3)) - mu_hat(0)*sin(mu_hat(3))*sin(mu_hat(5))
            + lm_odom.getX()*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4)) + lm_odom.getY()*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5))
            - mu_hat(0)*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4)) - mu_hat(1)*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5));
    H_(0,4) = -sin(mu_hat(3))*(lm_odom.getZ()*sin(mu_hat(4)) - mu_hat(2)*sin(mu_hat(4)) - lm_odom.getX()*cos(mu_hat(4))*cos(mu_hat(5))
              - lm_odom.getY()*cos(mu_hat(4))*sin(mu_hat(5)) + mu_hat(0)*cos(mu_hat(4))*cos(mu_hat(5)) + mu_hat(1)*cos(mu_hat(4))*sin(mu_hat(5)));
    H_(0,5) = mu_hat(0)*cos(mu_hat(3))*cos(mu_hat(5)) - lm_odom.getY()*cos(mu_hat(3))*sin(mu_hat(5)) - lm_odom.getX()*cos(mu_hat(3))*cos(mu_hat(5))
            + mu_hat(1)*cos(mu_hat(3))*sin(mu_hat(5)) + lm_odom.getY()*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3))
            - lm_odom.getX()*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5)) - mu_hat(1)*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3))
            + mu_hat(0)*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));

    H_(1,0) = - sin(mu_hat(3))*sin(mu_hat(5)) - cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4));
    H_(1,1) = cos(mu_hat(5))*sin(mu_hat(3)) - cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5));
    H_(1,2) = -cos(mu_hat(4))*cos(mu_hat(3));
    H_(1,3) = lm_odom.getX()*cos(mu_hat(3))*sin(mu_hat(5)) - lm_odom.getZ()*cos(mu_hat(4))*sin(mu_hat(3)) - lm_odom.getY()*cos(mu_hat(3))*cos(mu_hat(5))
            + mu_hat(1)*cos(mu_hat(3))*cos(mu_hat(5)) + mu_hat(2)*cos(mu_hat(4))*sin(mu_hat(3)) - mu_hat(0)*cos(mu_hat(3))*sin(mu_hat(5))
            - lm_odom.getX()*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3)) - lm_odom.getY()*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5))
            + mu_hat(0)*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3)) + mu_hat(1)*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));
    H_(1,4) = -cos(mu_hat(3))*(lm_odom.getZ()*sin(mu_hat(4)) - mu_hat(2)*sin(mu_hat(4)) - lm_odom.getX()*cos(mu_hat(4))*cos(mu_hat(5))
              - lm_odom.getY()*cos(mu_hat(4))*sin(mu_hat(5)) + mu_hat(0)*cos(mu_hat(4))*cos(mu_hat(5)) + mu_hat(1)*cos(mu_hat(4))*sin(mu_hat(5)));
    H_(1,5) = lm_odom.getX()*cos(mu_hat(5))*sin(mu_hat(3)) + lm_odom.getY()*sin(mu_hat(3))*sin(mu_hat(5)) - mu_hat(0)*cos(mu_hat(5))*sin(mu_hat(3))
            - mu_hat(1)*sin(mu_hat(3))*sin(mu_hat(5)) + lm_odom.getY()*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4))
            - lm_odom.getX()*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5)) - mu_hat(1)*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4))
            + mu_hat(0)*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5));
}

void CorrespondenceClass::computeS(const boost::numeric::ublas::matrix<double> &sigma,
                          const boost::numeric::ublas::matrix<double> &Q){
    // Intermidiate steps
    boost::numeric::ublas::matrix<double> mat = boost::numeric::ublas::prod(H_, sigma);
    boost::numeric::ublas::matrix<double> mat1 = boost::numeric::ublas::trans(H_);
    // Computation of S
    S_ = boost::numeric::ublas::prod(mat, mat1);
    S_ += Q;
}

void CorrespondenceClass::computeNu(const boost::numeric::ublas::vector<double> &z_hat_i,
                           const boost::numeric::ublas::vector<double> &z_i){
    nu_ = boost::numeric::ublas::vector<double>(2);
    // The innovation is only considered in y and z
    nu_(0) = z_i(1) - z_hat_i(1);
    nu_(1) = z_i(2) - z_hat_i(2);
}

void CorrespondenceClass::computeLikelihood(){
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
