#ifndef SMALL_AUV_NAV_HPP
#define SMALL_AUV_NAV_HPP

#include "utils_matrices.hpp"

#include <queue>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/thread/mutex.hpp>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/tf.h>


class LoLoEKF{

public:

    LoLoEKF(std::string node_name, ros::NodeHandle &nh);
    void ekfLocalize();
    ~LoLoEKF();

private:

    ros::Subscriber imu_subs_;
    ros::Subscriber dvl_subs_;
    ros::Subscriber tf_gt_subs_;
    ros::Publisher odom_pub_;

    // Handlers for sensors
    std::queue<sensor_msgs::ImuPtr> imu_readings_;
    std::queue<geometry_msgs::TwistWithCovarianceStampedPtr> dvl_readings_;
    std::queue<nav_msgs::OdometryPtr> gt_readings_;
    boost::mutex msg_lock_;

    void imuCB(const sensor_msgs::ImuPtr &imu_msg);
    void dvlCB(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg);
    void gtCB(const nav_msgs::OdometryPtr &pose_msg);

    boost::numeric::ublas::matrix<double> sigma_imu_;
    boost::numeric::ublas::vector<double> mu_imu_ ;

    double angleLimit (double angle) const;

    // EKF methods
    void init(const double &initial_R, const double &initial_Q, const double &delta, const unsigned int &size_n, const unsigned int &size_k);
    void computeOdom(const sensor_msgs::ImuPtr sensor_reading);
    void predictionStep(const sensor_msgs::ImuPtr sensor_reading);
    void predictMeasurementModel();
    void dataAssociation();
    void sequentialUpdate();
    void batchUpdate();

};

#endif // SMALL_AUV_NAV_HPP
