#include "utils/ekf_utils.hpp"

namespace utils{

    void updateMatrixBlock(const Eigen::MatrixXd& sigma_in, Eigen::MatrixXd& sigma_out, int lm_num){
        sigma_out.block(6,0,3,6) = sigma_in.block(lm_num * 3 + 6, 0, 3, 6);
        sigma_out.block(0,6,6,3) = sigma_in.block(0, lm_num * 3 + 6, 6, 3);
        sigma_out.block(6,6,3,3) = sigma_in.block(lm_num * 3 + 6, lm_num * 3 + 6, 3, 3);
    }

    void addLMtoFilter(Eigen::VectorXd& mu_hat, Eigen::MatrixXd& Sigma_hat, Eigen::Vector3d landmark, const std::tuple<double, double, double>& sigma_new){

        // Add new possible landmark to mu_hat
        Eigen::VectorXd aux_mu = mu_hat;
        mu_hat.resize(mu_hat.size()+3, true);
        mu_hat << aux_mu, landmark;

        // Increase Sigma_hat
        addLMtoMatrix(Sigma_hat, sigma_new);
    }

    void addLMtoMatrix(Eigen::MatrixXd &matrix, const std::tuple<double, double, double> &sigma_new){
        // Increase matrix
        matrix.conservativeResize(matrix.rows()+3, matrix.cols()+3);
        matrix.bottomRows(3).setZero();
        matrix.rightCols(3).setZero();
        matrix(matrix.rows()-3, matrix.cols()-3) = std::get<0>(sigma_new);  // TODO: initialize with uncertainty on the measurement in x,y,z
        matrix(matrix.rows()-2, matrix.cols()-2) = std::get<1>(sigma_new);
        matrix(matrix.rows()-1, matrix.cols()-1) = std::get<2>(sigma_new);
    }

    void removeLMfromFilter(Eigen::VectorXd &mu_hat, Eigen::MatrixXd &Sigma_hat, int j){

    }

    double angleLimit (double angle){ // keep angle within [-pi;pi)
            return std::fmod(angle + M_PI, (M_PI * 2)) - M_PI;
    }
}
