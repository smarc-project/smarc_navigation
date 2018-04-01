#ifndef CORRESPONDENCECLASS_HPP
#define CORRESPONDENCECLASS_HPP

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

#include <boost/scoped_ptr.hpp>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/inverse_chi_squared.hpp>

#include <vector>
#include <fstream>
#include <queue>
#include <math.h>
#include <algorithm>
#include <functional>
#include <list>
#include <iostream>
#include <cctype>

#include "utils_matrices/utils_matrices.hpp"

/**
 * @brief The CorrespondenceClass class
 * Auxiliar class containing all the info related to a correspondence
 * made between a landmark j and a measurement i at time t
 */

struct jacobian_components{
    double mu_0;
    double mu_1;
    double mu_2;
    double c_3;
    double c_4;
    double c_5;
    double s_3;
    double s_4;
    double s_5;
    Eigen::Matrix3d R_fls_base_;
};

typedef struct jacobian_components h_comp;

class CorrespondenceClass{

public:

    double psi_;
    double d_m_;
    Eigen::MatrixXd H_t_;
    Eigen::MatrixXd h_1_;
    Eigen::MatrixXd S_inverted_;
    Eigen::Vector3d landmark_pos_;
    Eigen::VectorXd nu_;
    std::pair<int, double> i_j_;

    CorrespondenceClass(){}

    CorrespondenceClass(const int &, const double &){}
    /**
     * @brief computeH
     * @param mu_hat
     * @param lm_odom
     * Computes the jacobian of the measurment model
     */

    CorrespondenceClass(CorrespondenceClass&&) = default; // forces a move constructor despite having a destructor
    CorrespondenceClass& operator=(CorrespondenceClass&&) = default; // force a move assignment anyway

    virtual ~CorrespondenceClass(){}

    virtual std::tuple<Eigen::Vector3d, Eigen::Vector3d> measModel(const tf::Vector3&, const tf::Transform&){
        return std::make_tuple(Eigen::Vector3d(), Eigen::Vector3d());
    }

    virtual Eigen::VectorXd backProjectNewLM(const Eigen::VectorXd&, const tf::Transform&){
        return Eigen::VectorXd();
    }

    virtual void computeH(const h_comp ,
                  const tf::Vector3 , const Eigen::Vector3d ){}
    /**
     * @brief computeS
     * @param sigma
     * @param Q
     * S = H*Q*H^T + Q
     */
    virtual void computeMHLDistance(const Eigen::MatrixXd &, const Eigen::MatrixXd &){}
    /**
     * @brief computeNu
     * @param z_hat_i
     * @param z_i
     * Computes the innovation
     */
    virtual void computeNu(const Eigen::Vector3d &, const Eigen::Vector3d &){}
    /**
     * @brief computeLikelihood
     * Likelihood of the correspondence mj, zi
     * It also outputs the Mahalanobis distance between z_hat_i, z_i for outlier detection
     */
    virtual void computeLikelihood(){}

private:
};

#endif // CORRESPONDENCECLASS_HPP
