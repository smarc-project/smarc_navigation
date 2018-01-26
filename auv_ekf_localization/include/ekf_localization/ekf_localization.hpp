#ifndef EKF_LOCALIZATION_HPP
#define EKF_LOCALIZATION_HPP

#include <ros/timer.h>
#include <ros/ros.h>

#include "utils_matrices/utils_matrices.hpp"
#include "auv_ekf_localization/map_ekf.h"
#include "correspondence_class/correspondence_class.hpp"

#include <queue>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <ros/transport_hints.h>
#include <boost/scoped_ptr.hpp>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/inverse_chi_squared.hpp>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TwistWithCovarianceStamped> MsgTimingPolicy;

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
    void init();

private:

    // ROS variables
    ros::NodeHandle *nh_;
    std::string node_name_;
    message_filters::Synchronizer<MsgTimingPolicy>* msg_synch_ptr_;
    message_filters::Subscriber<sensor_msgs::Imu>* imu_subs_;
    message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>* dvl_subs_;
    ros::Timer timer_;

    // Comms
    ros::Subscriber fast_imu_sub_;
    ros::Subscriber fast_dvl_sub_;
    ros::Subscriber tf_gt_subs_;
    ros::Subscriber observs_subs_;
    ros::Subscriber rpt_subs_;
    ros::Publisher odom_pub_;
    ros::ServiceClient map_client_;
    ros::Publisher vis_pub_;
    visualization_msgs::MarkerArray markers_;

    // Handlers for sensors
    std::deque<sensor_msgs::ImuPtr> imu_readings_; // TODO: add limit size to queues
    std::deque<geometry_msgs::TwistWithCovarianceStampedPtr> dvl_readings_;
    std::deque<nav_msgs::OdometryPtr> gt_readings_;
    std::deque<geometry_msgs::PointStampedPtr> measurements_t_;
    boost::mutex msg_lock_;
    std::vector<boost::numeric::ublas::vector<double>> map_;
    bool init_filter_;

    // System state variables
    boost::numeric::ublas::vector<double> mu_;
    boost::numeric::ublas::vector<double> mu_hat_;
    boost::numeric::ublas::matrix<double> Sigma_;
    boost::numeric::ublas::matrix<double> Sigma_hat_;
    boost::numeric::ublas::matrix<double> G_t_;
    double delta_m_;
    double lambda_M_;

    // Noise models
    boost::numeric::ublas::matrix<double> R_;
    boost::numeric::ublas::matrix<double> Q_;

    // Aux
    double t_prev_;
    bool coord_;
    unsigned int size_imu_q_;
    unsigned int size_dvl_q_;

    // tf
    tf::TransformBroadcaster odom_bc_;
    tf::StampedTransform transf_dvl_base_;
    tf::StampedTransform transf_world_odom_;    
    tf::Transform transf_odom_world_;
    tf::StampedTransform transf_base_sssr_;
    std::string odom_frame_;
    std::string world_frame_;
    std::string base_frame_;
    std::string dvl_frame_;
    std::string map_srv_name_;
    std::string sssr_frame_;

    // Input callbacks
    void gtCB(const nav_msgs::OdometryPtr &pose_msg);
    void synchSensorsCB(const sensor_msgs::ImuConstPtr &imu_msg,
                        const geometry_msgs::TwistWithCovarianceStampedConstPtr &dvl_msg);
    void fastIMUCB(const sensor_msgs::ImuPtr &imu_msg);
    void fastDVLCB(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg);
    void observationsCB(const geometry_msgs::PointStampedPtr &observ_msg);

//    void rptCB(const geometry_msgs::PoseWithCovarianceStampedPtr & ptr_msg);

    /**
     * @brief EKFLocalization::computeOdom
     * @param dvl_msg
     * @param gt_pose
     * @param q_auv
     * @param u_t
     * Integrates IMU and DVL to predict an estimate of the pose
     */
    void computeOdom(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg,
                     const nav_msgs::OdometryPtr &gt_pose, const tf::Quaternion &q_auv,
                     boost::numeric::ublas::vector<double> &u_t);


    /**
     * @brief EKFLocalization::predictMotion
     * @param u_t
     * Prediction step for the EKF
     */
    void predictMotion(boost::numeric::ublas::vector<double> &u_t);

    /**
     * @brief predictMeasurement
     * @param landmark_j
     * @param z_i
     * @param ml_i_list
     * Measurement prediction for a given pair measurement-landmark at time t
     */
    void predictMeasurement(const boost::numeric::ublas::vector<double> &landmark_j,
                                 boost::numeric::ublas::vector<double> &z_i,
                                 std::vector<CorrespondenceClass *> &ml_i_list);

    /**
     * @brief dataAssociation
     * Maximum likelihood data association with outlier rejection
     */
    void dataAssociation();

    /**
     * @brief sequentialUpdate
     * @param c_i_j
     * Sequential update for a given match observation-landmark
     */
    void sequentialUpdate(CorrespondenceClass *c_i_j);


    /**
     * @brief createMapMarkers
     * Publishes the map as an array of markers for visualization in RVIZ
     */
    void createMapMarkers();

    /**
     * @brief EKFLocalization::sendOutput
     * @param t
     * @return
     * Publishes AUV odometry info and tf odom --> base_link
     */
    bool sendOutput(ros::Time t);

    /**
     * @brief EKFLocalization::interpolateDVL
     * @param t_now
     * @param dvl_msg_ptr
     * Interpolates DVL (slower) inputs through Bezier curves to synch to faster sensors
     */
    void interpolateDVL(ros::Time t_now, geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg_ptr);


};

#endif // EKF_LOCALIZATION_HPP
