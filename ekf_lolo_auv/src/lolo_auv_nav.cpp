#include "lolo_auv_nav/lolo_auv_nav.hpp"

LoLoEKF::LoLoEKF(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh){

    // TODO_NACHO: Create a flexible sensors msgs container to be used in ekfLocalize() inheritated attribute]
    std::string imu_topic;
    std::string dvl_topic;
    std::string odom_topic;
    std::string gt_topic;

    nh_->param<std::string>((ros::this_node::getName() + "/imu_topic"), imu_topic, "/imu");
    nh_->param<std::string>((ros::this_node::getName() + "/dvl_topic"), dvl_topic, "/dvl");
    nh_->param<std::string>((ros::this_node::getName() + "/odom_pub_topic"), odom_topic, "/odom_ekf");
    nh_->param<std::string>((ros::this_node::getName() + "/gt_pose_topic"), gt_topic, "/gt_pose");

    // Node connections
    imu_subs_ = nh_->subscribe(imu_topic, 1, &LoLoEKF::imuCB, this);
    dvl_subs_ = nh_->subscribe(dvl_topic, 1, &LoLoEKF::dvlCB, this);
    tf_gt_subs_ = nh_->subscribe(gt_topic, 1, &LoLoEKF::gtCB, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic, 10);

    // TODO: necessary to clear up queue at init?
    while(!this->imu_readings_.empty()){
        imu_readings_.pop();
    }
}


double LoLoEKF::angleLimit (double angle) const{ // keep angle within [-pi;pi)
        return std::fmod(angle + M_PI, (M_PI * 2)) - M_PI;
}

// Freq is 50Hz
void LoLoEKF::imuCB(const sensor_msgs::ImuPtr &imu_msg){
    using namespace boost::numeric::ublas;
    boost::mutex::scoped_lock(msg_lock_);
    imu_readings_.push(imu_msg);
}

void LoLoEKF::gtCB(const nav_msgs::OdometryPtr &pose_msg){
    gt_readings_.push(pose_msg);
}


// Freq is 10Hz
void LoLoEKF::dvlCB(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg){
    dvl_readings_.push(dvl_msg);
}

double angle_diff(double angle_new, double angle_old) // return signed difference between new and old angle
{
    double diff = angle_new - angle_old;
    while (diff < -M_PI)
        diff += M_PI * 2;
    while (diff > M_PI)
        diff -= M_PI * 2;
    return diff;
}

void LoLoEKF::ekfLocalize(){
    ros::Rate rate(10);

    sensor_msgs::ImuPtr imu_msg;
    geometry_msgs::TwistWithCovarianceStampedPtr dvl_msg;
    nav_msgs::OdometryPtr gt_msg;

    double delta_t;
    double t_now;
    double t_prev;
    mu_ = boost::numeric::ublas::zero_vector<double>(6);
    nav_msgs::Odometry odom_msg;
    tf::TransformBroadcaster odom_bc;

    tf::TransformListener listener;
    tf::StampedTransform transf_dvl_base;
    tf::StampedTransform transf_world_odom;
    geometry_msgs::TransformStamped odom_trans;

    ROS_INFO("Initialized");
    ROS_INFO("-------------------------");
    bool loop = true;
    bool init_filter = true;
    double prev_imu_yaw;
    double theta;

    while(ros::ok()&& loop){
        ros::spinOnce();
        if(!dvl_readings_.empty() && !imu_readings_.empty() && !gt_readings_.empty()){

            // Init filter with initial, true pose (from GPS?)
            if(init_filter){
                ROS_INFO("Starting...");
                gt_msg = gt_readings_.back();
                t_prev = gt_msg->header.stamp.toSec();

                // Compute initial pose
                double qx = gt_msg->pose.pose.orientation.x;
                double qy = gt_msg->pose.pose.orientation.y;
                double qz = gt_msg->pose.pose.orientation.z;
                double qw = gt_msg->pose.pose.orientation.w;
                prev_imu_yaw = std::atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);
                theta = prev_imu_yaw;

                // Broadcast transform over tf
                odom_trans.header.stamp = gt_msg->header.stamp;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "lolo_auv/base_link";
                odom_trans.transform.translation.x = 0;
                odom_trans.transform.translation.y = 0;
                odom_trans.transform.translation.z = 0;
                odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
                odom_bc.sendTransform(odom_trans);

                // Publish odom msg
                odom_msg.header.stamp = gt_msg->header.stamp;
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "lolo_auv/base_link";
                odom_msg.pose.pose.position.x = 0;
                odom_msg.pose.pose.position.y = 0;
                odom_msg.pose.pose.position.z = 0;
                odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                odom_pub_.publish(odom_msg);

                // Get fixed transform from dvl to the base_link frame
                try {
                  listener.lookupTransform("lolo_auv/base_link", "lolo_auv/dvl_link", ros::Time(0), transf_dvl_base);
                  listener.waitForTransform("lolo_auv/base_link", "lolo_auv/dvl_link", ros::Time(0), ros::Duration(10.0) );
                  ROS_INFO("Locked transform dvl --> base");
                }
                catch(tf::TransformException &exception) {
                  ROS_ERROR("%s", exception.what());
                  ros::Duration(1.0).sleep();
                }

                try {
                  listener.lookupTransform("world", "odom", ros::Time(0), transf_world_odom);
                  listener.waitForTransform("world", "odom", ros::Time(0), ros::Duration(10.0) );
                  ROS_INFO("Locked transform world --> odom");
                }
                catch(tf::TransformException &exception) {
                  ROS_ERROR("%s", exception.what());
                  ros::Duration(1.0).sleep();
                }

                init_filter = false;
                continue;
            }

            // Integrate pose from dvl and imu
            imu_msg = imu_readings_.back();
            dvl_msg = dvl_readings_.back();
            gt_msg = gt_readings_.back();

            // Obtain absolute yaw from quaternion
            double qx = imu_msg->orientation.x;
            double qy = imu_msg->orientation.y;
            double qz = imu_msg->orientation.z;
            double qw = imu_msg->orientation.w;
            double imu_yaw = std::atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);
            double dtheta = angle_diff(imu_yaw, prev_imu_yaw);
            prev_imu_yaw = imu_yaw;

            // Compute norm of differential displacement
            t_now = dvl_msg->header.stamp.toSec();
            delta_t = t_now - t_prev;
            double disp = std::sqrt(pow((dvl_msg->twist.twist.linear.z * delta_t),2) + pow((dvl_msg->twist.twist.linear.y * delta_t),2));

            // Update pose xt = xt-1 + ut
            mu_(0) += std::cos(dtheta) * disp;
            mu_(1) += std::sin(dtheta) * disp;
            mu_(2) = gt_msg->pose.pose.position.z - transf_world_odom.getOrigin().getZ(); // Imitate depth sensor input
            theta += dtheta;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

            // Publish odom msg
            odom_msg.header.stamp = imu_msg->header.stamp;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "lolo_auv/base_link";
            odom_msg.pose.pose.position.x = mu_(0);
            odom_msg.pose.pose.position.y = mu_(1);
            odom_msg.pose.pose.position.z = mu_(2);
            odom_msg.pose.pose.orientation = odom_quat;
            odom_pub_.publish(odom_msg);

            // Broadcast transform over tf
            odom_trans.header.stamp = imu_msg->header.stamp;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "lolo_auv/base_link";
            odom_trans.transform.translation.x = mu_(0);
            odom_trans.transform.translation.y = mu_(1);
            odom_trans.transform.translation.z = mu_(2);
            odom_trans.transform.rotation = odom_quat;

            odom_bc.sendTransform(odom_trans);

            t_prev = t_now;
        }
        else{
            ROS_INFO("Still waiting for some good, nice sensor readings...");
        }
        rate.sleep();
    }
}

LoLoEKF::~LoLoEKF(){
    // TODO_NACHO: do some cleaning here
}
