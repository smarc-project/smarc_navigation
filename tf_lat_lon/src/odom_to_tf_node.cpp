#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#//include <tf2_ros/transform_listener.h>
#//include <tf2_ros/static_transform_broadcaster.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

string robot_name;

void odom_callback(const nav_msgs::Odometry& odom)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = robot_name + "/odom";
    transformStamped.child_frame_id = robot_name + "/base_link";
    transformStamped.transform.translation.x = odom.pose.pose.position.x;
    transformStamped.transform.translation.y = odom.pose.pose.position.y;
    transformStamped.transform.translation.z = odom.pose.pose.position.z;
    transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;

    br.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_tf_node");

    ros::NodeHandle private_node("~");
    if (!private_node.hasParam("robot_name")) {
        ROS_ERROR("need robot_name as argument");
        return -1;
    }
    else {
        private_node.getParam("robot_name", robot_name);
    }

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("dr/odom", 10, &odom_callback);

    ros::spin();

    return 0;
}
