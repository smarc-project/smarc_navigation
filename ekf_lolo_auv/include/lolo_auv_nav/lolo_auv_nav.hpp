#ifndef SMALL_AUV_NAV_HPP
#define SMALL_AUV_NAV_HPP

#include "utils_matrices.hpp"
#include "ekf_lolo_auv/map_ekf.h"
#include "landmark_ml/landmark_ml.hpp"

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

#include <ros/timer.h>
#include <ros/ros.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TwistWithCovarianceStamped> MsgTimingPolicy;

class LoLoEKF{

public:

    LoLoEKF(std::string node_name, ros::NodeHandle &nh);
    void ekfLocalize(const ros::TimerEvent& e);
    ~LoLoEKF();
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
    std::deque<geometry_msgs::PointStampedPtr> observ_readings_;
    boost::mutex msg_lock_;
    std::vector<boost::numeric::ublas::vector<double>> map_;
    bool init_filter_;

    // System state variables
    boost::numeric::ublas::vector<double> mu_;
    boost::numeric::ublas::vector<double> mu_hat_;
    boost::numeric::ublas::matrix<double> Sigma_;
    boost::numeric::ublas::matrix<double> Sigma_hat_;
    boost::numeric::ublas::matrix<double> G_t_;
    // Noise models
    boost::numeric::ublas::matrix<double> R_;
    boost::numeric::ublas::matrix<double> Q_;

    // Aux
    double z_t_; // Aux until model extended to 6DOF
    double t_prev_;
    bool coord_;
    unsigned int size_imu_q_;
    unsigned int size_dvl_q_;

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
    void synchSensorsCB(const sensor_msgs::ImuConstPtr &imu_msg, const geometry_msgs::TwistWithCovarianceStampedConstPtr &dvl_msg);
    void fastIMUCB(const sensor_msgs::ImuPtr &imu_msg);
    void fastDVLCB(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg);
    void observationsCB(const geometry_msgs::PointStampedPtr &observ_msg);

//    void rptCB(const geometry_msgs::PoseWithCovarianceStampedPtr & ptr_msg);

    // EKF methods
    void computeOdom(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg, const tf::Quaternion q_auv,
                     boost::numeric::ublas::vector<double> &u_t);
    void prediction(boost::numeric::ublas::vector<double> &u_t);

    void predictMeasurementModel(const boost::numeric::ublas::vector<int> &landmark_j,
                                 boost::numeric::ublas::vector<double> &z_i,
                                 std::vector<LandmarkML *> &ml_i_list);
    void dataAssociation(std::vector<LandmarkML *> &ml_t_list);
    void sequentialUpdate(std::vector<LandmarkML *> &observ_list);

    void update();


    // Aux methods
    void createMapMarkers();
    void transIMUframe(const geometry_msgs::Quaternion &auv_quat, tf::Quaternion &q_auv);
    bool sendOutput(ros::Time t);
    double angleLimit (double angle) const;
    void interpolateDVL(ros::Time t_now, geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg_ptr);


};

#endif // SMALL_AUV_NAV_HPP
