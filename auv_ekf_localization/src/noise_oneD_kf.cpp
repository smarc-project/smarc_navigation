#include "noise_oneD_kf/noise_oneD_kf.hpp"


OneDKF::OneDKF(double mu_init, double sigma_init, double r, double q){

    // Initial params
    mu_ = mu_init;
    sigma_ = sigma_init;
    r_ = r;
    q_ = q;
}

double OneDKF::filter(double input){
    // Predict
    double mu_hat = mu_;
    double sigma_hat = sigma_ + r_;

    // Update
    double k_t = sigma_hat / (sigma_hat + q_);
    mu_ = mu_hat + k_t * (input - mu_hat);
    sigma_ = (1 - k_t) * sigma_hat;

    return mu_;
}
