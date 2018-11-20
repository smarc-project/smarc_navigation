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

#include "correspondence_class/correspondence_class.hpp"


CorrespondenceClass::CorrespondenceClass(const double &lm_id){
    landmark_id_ = lm_id;
}

void CorrespondenceClass::computeH(const boost::numeric::ublas::vector<double> &mu_hat,
                          const tf::Vector3 lm_odom){

    using namespace std;
    H_ = boost::numeric::ublas::identity_matrix<double>(3,6);

    H_(0,0) = -cos(mu_hat(4))*cos(mu_hat(5));
    H_(0,1) = -cos(mu_hat(4))*sin(mu_hat(5));
    H_(0,2) = sin(mu_hat(4));
    H_(0,3) = 0;
    H_(0,4) = mu_hat(2)*cos(mu_hat(4)) - lm_odom.getZ()*cos(mu_hat(4)) - lm_odom.getX()*cos(mu_hat(5))*sin(mu_hat(4)) - lm_odom.getY()*sin(mu_hat(4))
            *sin(mu_hat(5)) + mu_hat(0)*cos(mu_hat(5))*sin(mu_hat(4)) + mu_hat(1)*sin(mu_hat(4))*sin(mu_hat(5));
    H_(0,5) = cos(mu_hat(4))*(lm_odom.getY()*cos(mu_hat(5)) - lm_odom.getX()*sin(mu_hat(5)) - mu_hat(1)*cos(mu_hat(5)) + mu_hat(0)*sin(mu_hat(5)));

    H_(1,0) = cos(mu_hat(3))*sin(mu_hat(5)) - cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3));
    H_(1,1) = - cos(mu_hat(3))*cos(mu_hat(5)) - sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));
    H_(1,2) = -cos(mu_hat(4))*sin(mu_hat(3));
    H_(1,3) = lm_odom.getZ()*cos(mu_hat(4))*cos(mu_hat(3)) - mu_hat(2)*cos(mu_hat(4))*cos(mu_hat(3)) - lm_odom.getY()*cos(mu_hat(5))*sin(mu_hat(3))
            + lm_odom.getX()*sin(mu_hat(3))*sin(mu_hat(5)) + mu_hat(1)*cos(mu_hat(5))*sin(mu_hat(3)) - mu_hat(0)*sin(mu_hat(3))*sin(mu_hat(5))
            + lm_odom.getX()*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4)) + lm_odom.getY()*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5))
            - mu_hat(0)*cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4)) - mu_hat(1)*cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5));
    H_(1,4) = -sin(mu_hat(3))*(lm_odom.getZ()*sin(mu_hat(4)) - mu_hat(2)*sin(mu_hat(4)) - lm_odom.getX()*cos(mu_hat(4))*cos(mu_hat(5))
              - lm_odom.getY()*cos(mu_hat(4))*sin(mu_hat(5)) + mu_hat(0)*cos(mu_hat(4))*cos(mu_hat(5)) + mu_hat(1)*cos(mu_hat(4))*sin(mu_hat(5)));
    H_(1,5) = mu_hat(0)*cos(mu_hat(3))*cos(mu_hat(5)) - lm_odom.getY()*cos(mu_hat(3))*sin(mu_hat(5)) - lm_odom.getX()*cos(mu_hat(3))*cos(mu_hat(5))
            + mu_hat(1)*cos(mu_hat(3))*sin(mu_hat(5)) + lm_odom.getY()*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3))
            - lm_odom.getX()*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5)) - mu_hat(1)*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3))
            + mu_hat(0)*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));

    H_(2,0) = - sin(mu_hat(3))*sin(mu_hat(5)) - cos(mu_hat(3))*cos(mu_hat(5))*sin(mu_hat(4));
    H_(2,1) = cos(mu_hat(5))*sin(mu_hat(3)) - cos(mu_hat(3))*sin(mu_hat(4))*sin(mu_hat(5));
    H_(2,2) = -cos(mu_hat(4))*cos(mu_hat(3));
    H_(2,3) = lm_odom.getX()*cos(mu_hat(3))*sin(mu_hat(5)) - lm_odom.getZ()*cos(mu_hat(4))*sin(mu_hat(3)) - lm_odom.getY()*cos(mu_hat(3))*cos(mu_hat(5))
            + mu_hat(1)*cos(mu_hat(3))*cos(mu_hat(5)) + mu_hat(2)*cos(mu_hat(4))*sin(mu_hat(3)) - mu_hat(0)*cos(mu_hat(3))*sin(mu_hat(5))
            - lm_odom.getX()*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3)) - lm_odom.getY()*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5))
            + mu_hat(0)*cos(mu_hat(5))*sin(mu_hat(4))*sin(mu_hat(3)) + mu_hat(1)*sin(mu_hat(4))*sin(mu_hat(3))*sin(mu_hat(5));
    H_(2,4) = -cos(mu_hat(3))*(lm_odom.getZ()*sin(mu_hat(4)) - mu_hat(2)*sin(mu_hat(4)) - lm_odom.getX()*cos(mu_hat(4))*cos(mu_hat(5))
              - lm_odom.getY()*cos(mu_hat(4))*sin(mu_hat(5)) + mu_hat(0)*cos(mu_hat(4))*cos(mu_hat(5)) + mu_hat(1)*cos(mu_hat(4))*sin(mu_hat(5)));
    H_(2,5) = lm_odom.getX()*cos(mu_hat(5))*sin(mu_hat(3)) + lm_odom.getY()*sin(mu_hat(3))*sin(mu_hat(5)) - mu_hat(0)*cos(mu_hat(5))*sin(mu_hat(3))
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
    nu_ = boost::numeric::ublas::vector<double>(z_i.size());
    nu_ = z_i - z_hat_i;
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
