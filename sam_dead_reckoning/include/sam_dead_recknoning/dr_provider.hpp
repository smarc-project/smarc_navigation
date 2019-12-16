#ifndef DR_PROVIDER_HPP
#define DR_PROVIDER_HPP

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/LinearMath/Transform.h"
#include <tf/transform_datatypes.h>

class drNode{

public:
    drNode(std::string node_name, ros::NodeHandle &nh);
    // ~drNode();

    ros::NodeHandle* nh_;
    std::string node_name_;
    ros::Subscriber imu_subs_;
    ros::Publisher odom_pub_;

    double gAcc_;
    double odom_x_;
    double odom_y_;
    double odom_z_;
    double t_prev_;

    void imuCB(const sensor_msgs::Imu::ConstPtr& imu_msg);

};

#endif // DR_PROVIDER_HPP
