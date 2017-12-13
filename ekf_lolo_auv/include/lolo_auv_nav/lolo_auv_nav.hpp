#ifndef SMALL_AUV_NAV_HPP
#define SMALL_AUV_NAV_HPP

#include "utils_matrices.hpp"

#include <queue>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <ros/transport_hints.h>
#include <boost/scoped_ptr.hpp>



#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "ekf_lolo_auv/map_ekf.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TwistWithCovarianceStamped> MsgTimingPolicy;

class LoLoEKF{

public:

    LoLoEKF(std::string node_name, ros::NodeHandle &nh);
    void ekfLocalize();
    ~LoLoEKF();
    void init();

private:

    // ROS variables
    ros::NodeHandle *nh_;
    std::string node_name_;
    message_filters::Synchronizer<MsgTimingPolicy>* msg_synch_ptr_;
    message_filters::Subscriber<sensor_msgs::Imu>* imu_subs_;
    message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>* dvl_subs_;

    ros::Subscriber tf_gt_subs_;
    ros::Subscriber rpt_subs_;
    ros::Publisher odom_pub_;
    ros::ServiceClient map_client_;
    ros::Timer timer_;
    ros::Publisher vis_pub_;

    visualization_msgs::MarkerArray markers_;

    // Handlers for sensors
    std::queue<sensor_msgs::ImuConstPtr> imu_readings_;
    std::queue<geometry_msgs::TwistWithCovarianceStampedConstPtr> dvl_readings_;
    std::queue<nav_msgs::OdometryPtr> gt_readings_;
    boost::mutex msg_lock_;
    std::vector<boost::numeric::ublas::vector<double>> map_;

    // System state variables
    boost::numeric::ublas::vector<double> mu_;
    boost::numeric::ublas::vector<double> mu_hat_;
    boost::numeric::ublas::matrix<double> Sigma_;
    boost::numeric::ublas::matrix<double> Sigma_hat_;
    boost::numeric::ublas::matrix<double> G_t_;
    // Noise models
    boost::numeric::ublas::matrix<double> R_;
    boost::numeric::ublas::matrix<double> Q_;

    double delta_t_;
    double z_t_; // Aux until model extended to 6DOF
    // tf
    tf::TransformBroadcaster odom_bc_;
    tf::StampedTransform transf_dvl_base_;
    tf::StampedTransform transf_world_odom_;
    std::string odom_frame_;
    std::string world_frame_;
    std::string base_frame_;
    std::string dvl_frame_;
    std::string map_srv_name_;

    // Callbacks
    void gtCB(const nav_msgs::OdometryPtr &pose_msg);
    void rptCB(const geometry_msgs::PoseWithCovarianceStampedPtr & ptr_msg);

    // EKF methods
    void computeOdom(const geometry_msgs::TwistWithCovarianceStampedConstPtr &dvl_msg, const tf::Quaternion q_auv,
                     double &t_prev, boost::numeric::ublas::vector<double> &u_t);
    void prediction(boost::numeric::ublas::vector<double> &u_t);
    void update();


    // Aux methods
    void synchSensorsCB(const sensor_msgs::ImuConstPtr &imu_msg, const geometry_msgs::TwistWithCovarianceStampedConstPtr &dvl_msg);
    void createMapMarkers();
    void transIMUframe(const geometry_msgs::Quaternion &auv_quat, tf::Quaternion &q_auv);
    bool sendOutput(ros::Time t);
    double angleLimit (double angle) const;
};

#endif // SMALL_AUV_NAV_HPP
