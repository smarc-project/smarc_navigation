#ifndef NOISE_ONED_KF_HPP
#define NOISE_ONED_KF_HPP

#include "ros/ros.h"

class OneDKF{

public:
    OneDKF(double mu_init, double sigma_init, double r, double q);
    ~OneDKF();
    double filter(double input);

private:

    double mu_hat_;
    double sigma_hat_;
    double mu_;
    double sigma_;
    double r_;
    double q_;
};



#endif
