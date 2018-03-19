#ifndef EKF_LOCALIZATION_HPP
#define EKF_LOCALIZATION_HPP

#include <ros/timer.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <Eigen/SparseCore>

#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/GetModelState.h"

#include "correspondence_class/correspondence_class.hpp"
#include "noise_oneD_kf/noise_oneD_kf.hpp"

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


/**
 * @brief The EKFLocalization class
 * EKF-based localization node for LoLo
 * Inputs:
 * IMU, DVL and landmarks positions from measurements
 * Map as a collection of landmarks with respect to world frame
 * Outputs:
 * nav_msgs/Odometry with an estimate of the 6DOF pose of LoLo
 * updated tf transform odom --> base_link
 */

class EKFLocalization{

public:

    EKFLocalization(std::string node_name, ros::NodeHandle &nh);
    void ekfLocalize(const ros::TimerEvent& e);
    ~EKFLocalization();
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
    bool init_filter_;

    // System state variables
    Eigen::MatrixXd Sigma_;
    Eigen::MatrixXd Sigma_hat_;
    Eigen::VectorXd mu_;
    Eigen::VectorXd mu_hat_;
    Eigen::Vector3d mu_auv_odom_;

    double delta_m_;
    double lambda_M_;

    // Mapping variables
    int lm_num_;

    // Noise models
    Eigen::MatrixXd R_;
    Eigen::MatrixXd Q_;

    // Aux
    double t_prev_;
    bool coord_;
    int size_odom_q_;

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
     * @brief EKFLocalization::predictMotion
     * @param u_t
     * Prediction step for the EKF
     */
    void predictMotion(nav_msgs::Odometry odom_reading);

    /**
     * @brief predictMeasurement
     * @param landmark_j
     * @param z_i
     * @param ml_i_list
     * Measurement prediction for a given pair measurement-landmark at time t
     */
    void predictMeasurement(const Eigen::Vector3d &landmark_j,
                            const Eigen::Vector3d &z_i,
                            unsigned int i, unsigned int j, const tf::Transform &transf_base_odom, const Eigen::MatrixXd &temp_sigma, h_comp h_comps,
                            std::vector<CorrespondenceClass> &ml_i_list);

    /**
     * @brief dataAssociation
     * Maximum likelihood data association with outlier rejection
     */
    void dataAssociation(std::vector<Eigen::Vector3d> z_t);

    /**
     * @brief sequentialUpdate
     * @param c_i_j
     * Sequential update for a given match observation-landmark
     */
    void sequentialUpdate(const CorrespondenceClass &c_i_j, Eigen::MatrixXd temp_sigma);


    /**
     * @brief createMapMarkers
     * Publishes the map as an array of markers for visualization in RVIZ
     */
    void updateMapMarkers(double color);

    /**
     * @brief EKFLocalization::sendOutput
     * @param t
     * @return
     * Publishes AUV odometry info and tf odom --> base_link
     */
    bool sendOutput(ros::Time t);

    bool bcMapOdomTF(ros::Time t);

};

#endif // EKF_LOCALIZATION_HPP
