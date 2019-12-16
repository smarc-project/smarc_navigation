#include "sam_dead_recknoning/dr_provider.hpp"


drNode::drNode(std::string node_name, ros::NodeHandle &nh): 
nh_(&nh), node_name_(node_name){

    imu_subs_ = nh_->subscribe("/sam/core/stim_imu", 10, &drNode::imuCB, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>("imu_odom", 10);
    gAcc_ = 9.80665;
    odom_x_ = 0.0;
    odom_y_ = 0.0;
    odom_z_ = 0.0;
    t_prev_ = ros::Time::now().toSec();
}


// drNode::~drNode(){

// }

void drNode::imuCB(const sensor_msgs::Imu::ConstPtr &imu_msg){

    double t_now = ros::Time::now().toSec();
    double delta_t = t_now - t_prev_;

    tf2::Vector3 normAcc( 0, 0, gAcc_);
    tf2::Quaternion curAttitude;
    tf2::Transform trans;

    tf2::fromMsg(imu_msg->orientation, curAttitude);
    tf2::Vector3 accTmp(imu_msg->linear_acceleration.x,
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z);

    trans.setRotation(curAttitude);
    tf2::Vector3 rotNorm = trans.getBasis().inverse() * normAcc;
    accTmp.setX(accTmp.getX() - rotNorm.getX());
    accTmp.setY(accTmp.getY() - rotNorm.getY());
    accTmp.setZ(accTmp.getZ() - rotNorm.getZ());

    std::cout << "Acceleration before " 
    << rotNorm.getX() << " " << rotNorm.getY() << " " << rotNorm.getZ() <<
    " and after " 
    << accTmp.getX() <<" " << accTmp.getY() << " " << accTmp.getZ() << std::endl;

    // Integration
    odom_x_ += accTmp.getX() * (delta_t*delta_t); 
    odom_y_ += accTmp.getY() * (delta_t*delta_t); 
    odom_z_ += accTmp.getZ() * (delta_t*delta_t); 

    // Publish odom msg
    tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
    q_auv_t.normalize();
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q_auv_t, odom_quat);

    nav_msgs::Odometry odom_imul_msg;
    odom_imul_msg.header.stamp = ros::Time(t_now);
    odom_imul_msg.header.frame_id = "sam_odom";
    odom_imul_msg.child_frame_id = "sam/imu_link";
    odom_imul_msg.pose.pose.position.x = odom_x_;
    odom_imul_msg.pose.pose.position.y = odom_y_;
    odom_imul_msg.pose.pose.position.z = odom_z_;
    odom_imul_msg.pose.pose.orientation = odom_quat;
    odom_pub_.publish(odom_imul_msg);

    t_prev_ = t_now;
}