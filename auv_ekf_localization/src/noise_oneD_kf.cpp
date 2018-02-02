#include "noise_oneD_kf/noise_oneD_kf.hpp"


OneDKF::OneDKF(double mu_init, double sigma_init, double r, double q){

    // Initial params
    mu_ = mu_init;
    sigma_ = sigma_init;
    r_ = r;
    q_ = q;
}

void OneDKF::filter(double &input){
    // Predict
    mu_hat_ = mu_;
    sigma_hat_ = sigma_ + r_;

    // Update
    double k_t = sigma_hat_ / (sigma_hat_ + q_);
    mu_ = mu_hat_ + k_t * (input - mu_hat_);
    sigma_ = (1 - k_t) * sigma_hat_;

    input = mu_;
}
