#ifndef EKF_CLASS_HPP
#define EKF_CLASS_HPP


#include <ros/ros.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/operation_blocked.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

//#include <boost/scoped_ptr.hpp>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/inverse_chi_squared.hpp>

//#include <vector>
//#include <cctype>
//#include <fstream>
//#include <list>
//#include <iostream>
//#include <queue>

#include <algorithm>
#include <functional>
#include <math.h>


class GeneralEKF{
public:
    GeneralEKF(std::string node_name, ros::NodeHandle &nh);
    virtual ~GeneralEKF();
    void virtual ekfLocalize();

protected:
    // ROS variables
    ros::NodeHandle *nh_;
    std::string node_name_;
    // Sensors readings handlers

    // Problem definition variables
    int size_n_;
    int size_k_;
    bool sequential_update_;

    // State estimate variables
    boost::numeric::ublas::vector<double> mu_;
    boost::numeric::ublas::vector<double> mu_hat_;
    boost::numeric::ublas::matrix<double> sigma_;
    boost::numeric::ublas::matrix<double> sigma_hat_;
    // Noise covariances
    boost::numeric::ublas::matrix<double> R_;
    boost::numeric::ublas::matrix<double> Q_;
    // Measurement variables
    boost::numeric::ublas::vector<double> h_;
    // Outliers rejection variables
    double lambda_M_;
    double delta_m_;
    // Control variables
    boost::numeric::ublas::vector<double> u_t_;
    double t_prev_;
    double delta_t_;

    // EKF methods
    virtual void init(const double &initial_R, const double &initial_Q, const double &delta, const unsigned int &size_n, const unsigned int &size_k);
    virtual void computeOdom();
    virtual void predictionStep();
    virtual void predictMeasurementModel();
    virtual void dataAssociation();
    virtual void sequentialUpdate();
};

#endif // EKF_CLASS_HPP

