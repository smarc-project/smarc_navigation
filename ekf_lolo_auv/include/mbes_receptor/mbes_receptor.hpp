#ifndef MBES_RECEPTOR_HPP
#define MBES_RECEPTOR_HPP

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>


#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "sonar_manipulator/sonar_manipulator.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MyTimerPolicy;

class MBESReceptor{

public:
    MBESReceptor(std::string node_name, ros::NodeHandle &nh_right, ros::NodeHandle &nh_left);
    ~MBESReceptor();

private:
    std::string node_name_;
    ros::NodeHandle* nh_right_;
    ros::NodeHandle* nh_left_;
    ros::NodeHandle nh_;

    std::string sss_r_frame_;
    std::string sss_l_frame_;
    std::string base_frame_;
    tf::StampedTransform tf_sss_r_base_;
    tf::StampedTransform tf_sss_l_base_;

    message_filters::Subscriber<sensor_msgs::LaserScan>* mbes_l_subs_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* mbes_r_subs_;
    message_filters::Synchronizer<MyTimerPolicy>* mbes_synch_;
    ros::Publisher pcl_pub_;
    ros::Publisher landmark_pub_;

    // Create processors for sonar inputs
    SonarManipulator sss_right_;
    SonarManipulator sss_left_;

    // Methods
    void mbesReadingsCB(const sensor_msgs::LaserScanConstPtr & mbes_l_msg, const sensor_msgs::LaserScanConstPtr &mbes_r_msg );
    void init();
};


#endif // MBES_RECEPTOR_HPP
