#ifndef LANDMARK_ML_HPP
#define LANDMARK_ML_HPP

#include <ros/ros.h>
#include "ekf_general/sensors_read.h"
#include "ekf_general/plot_map.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/operation_blocked.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

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

#include "utils_matrices.hpp"

double angleLimit (double angle);


// Class for computation of ML given a landmark_j and an observation z_i_t
class LandmarkML{
public:

    LandmarkML(const boost::numeric::ublas::vector<int> &landmark_pos);
    void computeH(const boost::numeric::ublas::vector<double> &z_hat, const boost::numeric::ublas::vector<double> &mu_hat,
                  const tf::Transform world_base_tf);
    void computeS(const boost::numeric::ublas::matrix<double> &sigma, const boost::numeric::ublas::matrix<double> &Q);
    void computeNu(const boost::numeric::ublas::vector<double> &z_hat_i, const boost::numeric::ublas::vector<double> &z_i);
    void computeLikelihood();
    double psi_;
    double d_m_;
    boost::numeric::ublas::matrix<double> H_;
    boost::numeric::ublas::matrix<double> S_;
    boost::numeric::ublas::matrix<double> S_inverted_;
    boost::numeric::ublas::vector<int> landmark_pos_;
    boost::numeric::ublas::vector<double> nu_;
    int landmark_id_;

private:
};

#endif // LANDMARK_ML_HPP
