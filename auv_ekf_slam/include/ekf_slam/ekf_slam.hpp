#ifndef EKF_LOCALIZATION_HPP
#define EKF_LOCALIZATION_HPP

#include <ros/timer.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <Eigen/SparseCore>

#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/GetModelState.h"

#include <queue>
#include <math.h>

#include <boost/thread/mutex.hpp>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/inverse_chi_squared.hpp>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/tf.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "noise_oneD_kf/noise_oneD_kf.hpp"
#include "ekf_slam_core/ekf_slam_core.hpp"

/**
 * @brief The EKFSLAM class
 * EKF-based localization node for LoLo
 * Inputs:
 * IMU, DVL and landmarks positions from measurements
 * Map as a collection of landmarks with respect to world frame
 * Outputs:
 * nav_msgs/Odometry with an estimate of the 6DOF pose of LoLo
 * updated tf transform odom --> base_link
 */

class EKFSLAM{

public:

    EKFSLAM(std::string node_name, ros::NodeHandle &nh);
    void ekfLocalize(const ros::TimerEvent&);
    ~EKFSLAM();
    void init(std::vector<double> sigma_diag, std::vector<double> r_diag, std::vector<double> q_diag, double delta);

private:

    // ROS variables
    ros::NodeHandle *nh_;
    std::string node_name_;
    ros::Timer timer_;

    // Comms
    ros::Subscriber odom_subs_;
    ros::Subscriber observs_subs_;
    ros::Publisher map_pub_;
    ros::Publisher vis_pub_;
    ros::ServiceClient gazebo_client_;
    ros::ServiceClient landmarks_client_;

    // Handlers for sensors
    std::deque<geometry_msgs::PoseArray> measurements_t_;
    std::deque<nav_msgs::Odometry> odom_queue_t_;
    boost::mutex msg_lock_;

    // EKF state variables
    Eigen::VectorXd mu_;
    Eigen::MatrixXd Sigma_;
    EKFCore* ekf_filter_;

    // Mapping variables
    double lambda_M_;
    int lm_num_;

    // Noise models
    Eigen::MatrixXd R_;
    Eigen::MatrixXd Q_;

    // Aux
    unsigned int size_odom_q_;

    // tf
    tf::TransformBroadcaster map_bc_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform transf_dvl_base_;
    tf::StampedTransform transf_world_odom_;
    tf::Transform transf_odom_world_;
    tf::StampedTransform transf_base_sssr_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string world_frame_;
    std::string base_frame_;
    std::string sssr_frame_;
    std::string map_srv_name_;
    std::string lm_srv_name_;

    // Input callbacks
    void odomCB(const nav_msgs::Odometry &odom_msg);

    void observationsCB(const geometry_msgs::PoseArray &observ_msg);

    /**
     * @brief createMapMarkers
     * Publishes the map as an array of markers for visualization in RVIZ
     */
    void updateMapMarkers(double color);

    /**
     * @brief EKFSLAM::sendOutput
     * @param t
     * @return
     * Publishes AUV odometry info and tf odom --> base_link
     */
    bool sendOutput(ros::Time t);

    bool bcMapOdomTF(ros::Time t);

};

#endif // EKF_LOCALIZATION_HPP
