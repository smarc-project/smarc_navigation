#include "lolo_auv_nav/lolo_auv_nav.hpp"

LoLoEKF::LoLoEKF(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh){

    std::string imu_topic;
    std::string dvl_topic;
    std::string odom_topic;
    std::string gt_topic;

    nh_->param<std::string>((ros::this_node::getName() + "/imu_topic"), imu_topic, "/imu");
    nh_->param<std::string>((ros::this_node::getName() + "/dvl_topic"), dvl_topic, "/dvl");
    nh_->param<std::string>((ros::this_node::getName() + "/odom_pub_topic"), odom_topic, "/odom_ekf");
    nh_->param<std::string>((ros::this_node::getName() + "/gt_pose_topic"), gt_topic, "/gt_pose");
    nh_->param<std::string>((ros::this_node::getName() + "/odom_frame"), odom_frame_, "/odom");
    nh_->param<std::string>((ros::this_node::getName() + "/world_frame"), world_frame_, "/world");
    nh_->param<std::string>((ros::this_node::getName() + "/base_frame"), base_frame_, "/base_link");
    nh_->param<std::string>((ros::this_node::getName() + "/dvl_frame"), dvl_frame_, "/dvl_link");

    // Node connections
    imu_subs_ = nh_->subscribe(imu_topic, 1, &LoLoEKF::imuCB, this);
    dvl_subs_ = nh_->subscribe(dvl_topic, 1, &LoLoEKF::dvlCB, this);
    tf_gt_subs_ = nh_->subscribe(gt_topic, 1, &LoLoEKF::gtCB, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic, 10);
}


double LoLoEKF::angleLimit (double angle) const{ // keep angle within [-pi;pi)
        return std::fmod(angle + M_PI, (M_PI * 2)) - M_PI;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angleDiff (double angle_new, double angle_old) // return signed difference between new and old angle
{
    double diff = angle_new - angle_old;
    while (diff < -M_PI)
        diff += M_PI * 2;
    while (diff > M_PI)
        diff -= M_PI * 2;
    return diff;
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

bool LoLoEKF::sendOutput(ros::Time &t, geometry_msgs::Quaternion &odom_quat){

//    try{

        // Broadcast transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = t;
        odom_trans.header.frame_id = odom_frame_;
        odom_trans.child_frame_id = base_frame_;
        odom_trans.transform.translation.x = mu_(0);
        odom_trans.transform.translation.y = mu_(1);
        odom_trans.transform.translation.z = mu_(2);
        odom_trans.transform.rotation = odom_quat;
        odom_bc_.sendTransform(odom_trans);

        // Publish odom msg
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = t;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
        odom_msg.pose.pose.position.x = mu_(0);
        odom_msg.pose.pose.position.y = mu_(1);
        odom_msg.pose.pose.position.z = mu_(2);
        odom_msg.pose.pose.orientation = odom_quat;
        odom_pub_.publish(odom_msg);

        return true;
//    }
//    catch(){
//        ROS_ERROR("Odom update could not be sent");
//        return false;
//    }
}

void LoLoEKF::computeOdom(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg, const nav_msgs::OdometryPtr &gt_msg,
                          const sensor_msgs::ImuPtr &imu_msg, double &t_prev, double& yaw_prev, boost::numeric::ublas::vector<double> &u_t){

    // Update time step
    double t_now = dvl_msg->header.stamp.toSec();
    double delta_t = t_now - t_prev;

    // Transform from dvl input form dvl --> base_link frame
    tf::Vector3 twist_vel(dvl_msg->twist.twist.linear.x,
                          dvl_msg->twist.twist.linear.y,
                          dvl_msg->twist.twist.linear.z);

    tf::Vector3 l_vel_base = transf_dvl_base_.getBasis() * twist_vel;
    double w_z_base = transf_dvl_base_.getOrigin().getX() * dvl_msg->twist.twist.linear.y;

    // Transform IMU orientation from world to odom coordinates
    tf::Quaternion q_imu;
    tf::quaternionMsgToTF(imu_msg->orientation, q_imu);
    tf::Quaternion q_world_odom = transf_world_odom_.getRotation();
    tf::Quaternion q_auv = q_world_odom * q_imu;
    q_auv.normalize();  // TODO: implement handling of singularities?
    double yaw_t = tf::getYaw(q_auv);

    // Compute incremental displacement
    double disp = std::sqrt(pow((l_vel_base.y() * delta_t),2) +
                            pow((l_vel_base.x() * delta_t),2));

    double dtheta = std::atan2((l_vel_base.y() * delta_t),
                               (l_vel_base.x() * delta_t));
    double theta = angleLimit(yaw_prev + dtheta);

    // Compute control u_t
    int sign = (sgn(std::sin(theta) * disp) == 1)? -1: 1;
    u_t(0) = std::cos(theta) * disp;
    u_t(1) = std::sin(theta) * disp + sign * (w_z_base * delta_t) * transf_dvl_base_.getOrigin().getX();
    u_t(2) = gt_msg->pose.pose.position.z - transf_world_odom_.getOrigin().getZ(); // Imitate depth sensor input
    u_t(3) = yaw_t; // TODO: expand state space to 6DOF

    yaw_prev = yaw_t;
    t_prev = t_now;
}

void LoLoEKF::prediction(boost::numeric::ublas::vector<double> &u_t){
    mu_hat_(0) =+ u_t(0);
    mu_hat_(1) =+ u_t(1);
    mu_hat_(2) = u_t(2);
    mu_hat_(3) = u_t(3);
}

void LoLoEKF::update(){
    mu_ = mu_hat_;
}

void LoLoEKF::ekfLocalize(){
    ros::Rate rate(10);

    sensor_msgs::ImuPtr imu_msg;
    geometry_msgs::TwistWithCovarianceStampedPtr dvl_msg;
    nav_msgs::OdometryPtr gt_msg;

    double t_prev;
    double yaw_prev;
    // TODO: full implementation of 6 DOF movement
    mu_ = boost::numeric::ublas::zero_vector<double>(6);
    mu_hat_ = boost::numeric::ublas::zero_vector<double>(6);
    boost::numeric::ublas::vector<double> u_t = boost::numeric::ublas::vector<double>(6);

    bool loop = true;
    bool init_filter = true;

    while(ros::ok()&& loop){
        ros::spinOnce();
        if(!dvl_readings_.empty() && !imu_readings_.empty() && !gt_readings_.empty()){

            // Init filter with initial, true pose (from GPS?)
            if(init_filter){
                ROS_INFO("Starting localization node");

                // Get fixed transform dvl_link --> base_link frame
                tf::Quaternion q_world_odom;
                tf::TransformListener tf_listener;
                try {
                  tf_listener.lookupTransform(base_frame_, dvl_frame_, ros::Time(0), transf_dvl_base_);
                  tf_listener.waitForTransform(base_frame_, dvl_frame_, ros::Time(0), ros::Duration(10.0) );
                  ROS_INFO("Locked transform dvl --> base");
                }
                catch(tf::TransformException &exception) {
                  ROS_ERROR("%s", exception.what());
                  ros::Duration(1.0).sleep();
                }

                // Get fixed transform world --> odom frame
                try {
                  tf_listener.lookupTransform(world_frame_, odom_frame_, ros::Time(0), transf_world_odom_);
                  tf_listener.waitForTransform(world_frame_, odom_frame_, ros::Time(0), ros::Duration(10.0) );
                  q_world_odom = transf_world_odom_.getRotation();
                  ROS_INFO("Locked transform world --> odom");
                }
                catch(tf::TransformException &exception) {
                  ROS_ERROR("%s", exception.what());
                  ros::Duration(1.0).sleep();
                }

                // Compute initial pose
                gt_msg = gt_readings_.back();
                t_prev = gt_msg->header.stamp.toSec();
                tf::Quaternion q_gt;
                tf::quaternionMsgToTF(gt_msg->pose.pose.orientation, q_gt);
                tf::Quaternion q_auv = q_world_odom * q_gt;
                q_auv.normalize();
                yaw_prev = tf::getYaw(q_auv);

                // Publish and broadcast
                this->sendOutput(gt_msg->header.stamp, gt_msg->pose.pose.orientation);

                gt_readings_.pop();
                init_filter = false;
                continue;
            }

            // TODO: interpolate the faster sensors and adapt to slower ones
            // Integrate pose from dvl and imu
            imu_msg = imu_readings_.back();
            dvl_msg = dvl_readings_.back();
            gt_msg = gt_readings_.back();

            // Compute displacement based on DVL and IMU orientation
            computeOdom(dvl_msg, gt_msg, imu_msg, t_prev, yaw_prev, u_t);

            // Prediction step
            prediction(u_t);

            // Update step
            update();

            // Publish and broadcast
            this->sendOutput(dvl_msg->header.stamp, imu_msg->orientation);


            imu_readings_.pop();
            dvl_readings_.pop();
            gt_readings_.pop();
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
