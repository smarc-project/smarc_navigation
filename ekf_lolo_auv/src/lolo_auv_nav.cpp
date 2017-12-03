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

void LoLoEKF::ekfLocalize(){
    ros::Rate rate(10);

    sensor_msgs::ImuPtr imu_msg;
    geometry_msgs::TwistWithCovarianceStampedPtr dvl_msg;
    nav_msgs::OdometryPtr gt_msg;

    double delta_t;
    double t_now;
    double t_prev;
    // TODO: full implementation of 6 DOF movement
    mu_ = boost::numeric::ublas::zero_vector<double>(6);
    nav_msgs::Odometry odom_msg;

    tf::StampedTransform transf_dvl_base;
    tf::StampedTransform transf_world_odom;
    tf::TransformListener tf_listener;
    geometry_msgs::TransformStamped odom_trans;

    ROS_INFO("Initialized");
    ROS_INFO("-------------------------");
    bool loop = true;
    bool init_filter = true;

    double theta;
    tf::Quaternion q_imu;
    tf::Quaternion q_world_odom;
    geometry_msgs::Quaternion odom_quat;

    while(ros::ok()&& loop){
        ros::spinOnce();
        if(!dvl_readings_.empty() && !imu_readings_.empty() && !gt_readings_.empty()){

            // Init filter with initial, true pose (from GPS?)
            if(init_filter){
                ROS_INFO("Starting...");
                gt_msg = gt_readings_.back();
                t_prev = gt_msg->header.stamp.toSec();

                // TODO: substitute names by rosparams
                // Get fixed transform dvl_link --> base_link frame
                try {
                  tf_listener.lookupTransform("lolo_auv/base_link", "lolo_auv/dvl_link", ros::Time(0), transf_dvl_base);
                  tf_listener.waitForTransform("lolo_auv/base_link", "lolo_auv/dvl_link", ros::Time(0), ros::Duration(10.0) );
                  ROS_INFO("Locked transform dvl --> base");
                }
                catch(tf::TransformException &exception) {
                  ROS_ERROR("%s", exception.what());
                  ros::Duration(1.0).sleep();
                }

                // Get fixed transform world --> odom frame
                try {
                  tf_listener.lookupTransform("world", "odom", ros::Time(0), transf_world_odom);
                  tf_listener.waitForTransform("world", "odom", ros::Time(0), ros::Duration(10.0) );
                  q_world_odom = transf_world_odom.getRotation();
                  ROS_INFO("Locked transform world --> odom");
                }
                catch(tf::TransformException &exception) {
                  ROS_ERROR("%s", exception.what());
                  ros::Duration(1.0).sleep();
                }

                // Compute initial pose
                tf::Quaternion q_gt;
                tf::quaternionMsgToTF(gt_msg->pose.pose.orientation, q_gt);
                tf::Quaternion q_auv = q_world_odom * q_gt;
                q_auv.normalize();
                tf::quaternionTFToMsg(q_auv, odom_quat);

                // Broadcast transform over tf
                odom_trans.header.stamp = gt_msg->header.stamp;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "lolo_auv/base_link";
                odom_trans.transform.translation.x = mu_(0);
                odom_trans.transform.translation.y = mu_(1);
                odom_trans.transform.translation.z = mu_(2);
                odom_trans.transform.rotation = odom_quat;
                odom_bc_.sendTransform(odom_trans);

                // Publish odom msg
                odom_msg.header.stamp = gt_msg->header.stamp;
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "lolo_auv/base_link";
                odom_msg.pose.pose.position.x = mu_(0);
                odom_msg.pose.pose.position.y = mu_(1);
                odom_msg.pose.pose.position.z = mu_(2);
                odom_msg.pose.pose.orientation = odom_quat;
                odom_pub_.publish(odom_msg);

                init_filter = false;
                continue;
            }

            // TODO: interpolate the faster sensors and adapt to slower ones
            // Integrate pose from dvl and imu
            imu_msg = imu_readings_.back();
            dvl_msg = dvl_readings_.back();
            gt_msg = gt_readings_.back();

            // Transform IMU orientation from world to odom coordinates
            tf::quaternionMsgToTF(imu_msg->orientation, q_imu);
            q_world_odom.normalize();
            tf::Quaternion q_auv = q_world_odom * q_imu;
            q_auv.normalize();

            // Obtain absolute yaw from quaternion
            // TODO: implement handling for singularities
            double imu_yaw = std::atan2(2*(q_auv.x()*q_auv.y() + q_auv.w()*q_auv.z()),
                                        q_auv.w()*q_auv.w() + q_auv.x()*q_auv.x() - q_auv.y()*q_auv.y() - q_auv.z()*q_auv.z());

            theta = this->angleLimit(imu_yaw);

            // Update time step
            t_now = dvl_msg->header.stamp.toSec();
            delta_t = t_now - t_prev;
            t_prev = t_now;

            // Compute magnitud of differential displacement
            double disp = std::sqrt(pow((dvl_msg->twist.twist.linear.z * delta_t),2) +
                                    pow((dvl_msg->twist.twist.linear.y * delta_t),2));

            // Update pose xt = xt-1 + ut
            mu_(0) += std::cos(theta) * disp;
            mu_(1) += std::sin(theta) * disp;
            mu_(2) = gt_msg->pose.pose.position.z - transf_world_odom.getOrigin().getZ(); // Imitate depth sensor input
            odom_quat = tf::createQuaternionMsgFromYaw(theta);
            tf::quaternionTFToMsg(q_auv, odom_quat);

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
