#ifndef CORRESPONDENCE_FLS_CPP
#define CORRESPONDENCE_FLS_CPP

#include "correspondence_class/correspondence_class.hpp"

class CorrespondenceFLS: public CorrespondenceClass{

public:

    CorrespondenceFLS(const int &z_id, const double &lm_id);

    ~CorrespondenceFLS();

    void computeH(const h_comp h_comps, const tf::Vector3 lm_odom, const Eigen::Vector3d z_hat_fls_m);

    void computeMHLDistance(const Eigen::MatrixXd &sigma, const Eigen::MatrixXd &Q);

    void computeNu(const Eigen::Vector3d &z_hat_i, const Eigen::Vector3d &z_i);

    void computeLikelihood();

private:
};

#endif // CORRESPONDENCE_FLS_CPP
