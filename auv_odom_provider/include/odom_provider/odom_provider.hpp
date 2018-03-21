#ifndef ODOM_PROVIDER_HPP
#define ODOM_PROVIDER_HPP

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <ros/transport_hints.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,
        geometry_msgs::TwistWithCovarianceStamped> MsgTimingPolicy;

class OdomProvider{

public:
    OdomProvider(std::string node_name, ros::NodeHandle &nh);
    ~OdomProvider();
    void init();
    void provideOdom(const ros::TimerEvent& e);

private:
    ros::NodeHandle* nh_;
    std::string node_name_;
    message_filters::Synchronizer<MsgTimingPolicy>* msg_synch_ptr_;
    message_filters::Subscriber<sensor_msgs::Imu>* imu_subs_;
    message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>* dvl_subs_;

    Eigen::VectorXd cumul_odom_;

    // Comms
    ros::Subscriber fast_imu_sub_;
    ros::Subscriber fast_dvl_sub_;
    ros::Subscriber tf_gt_subs_;
    ros::Publisher odom_pub_;
    ros::Publisher odom_inertial_pub_;
    ros::Timer timer_;

    // Handlers for sensors
    std::deque<sensor_msgs::Imu> imu_readings_; // TODO: add limit size to queues
    std::deque<geometry_msgs::TwistWithCovarianceStamped> dvl_readings_;
    std::deque<nav_msgs::Odometry> gt_readings_;
    boost::mutex msg_lock_;
    bool init_filter_;

    // Aux
    double t_prev_;
    bool coord_;
    unsigned int size_imu_q_;
    unsigned int size_dvl_q_;

    // tf
    tf::TransformBroadcaster odom_bc_;
    tf::StampedTransform transf_base_dvl_;
    tf::StampedTransform transf_odom_world_;
    std::string odom_frame_;
    std::string world_frame_;
    std::string base_frame_;
    std::string dvl_frame_;

    // Input callbacks
    void gtCB(const nav_msgs::Odometry &pose_msg);
    void synchSensorsCB(const sensor_msgs::ImuConstPtr &imu_msg,
                        const geometry_msgs::TwistWithCovarianceStampedConstPtr &dvl_msg);
    void fastIMUCB(const sensor_msgs::Imu &imu_msg);
    void fastDVLCB(const geometry_msgs::TwistWithCovarianceStamped &dvl_msg);

    void interpolateDVL(ros::Time t_now, geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg_ptr);
    void computeOdom(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg, const tf::Quaternion &q_auv,
                     Eigen::VectorXd &u_t);

    bool sendOutput(ros::Time t);

};


#endif // ODOM_PROVIDER_HPP
