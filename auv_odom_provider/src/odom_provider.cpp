#include "odom_provider/odom_provider.hpp"

// HELPER FUNCTIONS TODO: move to aux library
unsigned int factorial(unsigned int n)
{
    unsigned int ret = 1;
    if(n == 0) return 1;
    for(unsigned int i = 1; i <= n; ++i){
        ret *= i;
    }
    return ret;
}

double angleLimit (double angle){ // keep angle within [-pi;pi)
        return std::fmod(angle + M_PI, (M_PI * 2)) - M_PI;
}

// END HELPER FUNCTIONS

OdomProvider::OdomProvider(std::string node_name, ros::NodeHandle &nh): nh_(&nh), node_name_(node_name){

    std::string imu_topic;
    std::string dvl_topic;
    std::string odom_topic;
    std::string odom_in_topic;
    std::string gt_topic;
    double freq;

    nh_->param<double>((node_name_ + "/system_freq"), freq, 30);
    nh_->param<std::string>((node_name_ + "/imu_topic"), imu_topic, "/imu");
    nh_->param<std::string>((node_name_ + "/dvl_topic"), dvl_topic, "/dvl");
    nh_->param<std::string>((node_name_ + "/odom_pub_topic"), odom_topic, "/odom_ekf");
    nh_->param<std::string>((node_name_ + "/gt_pose_topic"), gt_topic, "/gt_pose");
    nh_->param<std::string>((node_name_ + "/odom_frame"), odom_frame_, "/odom");
    nh_->param<std::string>((node_name_ + "/world_frame"), world_frame_, "/world");
    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/dvl_frame"), dvl_frame_, "/dvl_link");

    // Synch IMU and DVL readings
    imu_subs_ = new message_filters::Subscriber<sensor_msgs::Imu>(*nh_, imu_topic, 25);
    dvl_subs_ = new message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>(*nh_, dvl_topic, 5);
    msg_synch_ptr_ = new message_filters::Synchronizer<MsgTimingPolicy> (MsgTimingPolicy(5), *imu_subs_, *dvl_subs_);
    msg_synch_ptr_->registerCallback(boost::bind(&OdomProvider::synchSensorsCB, this, _1, _2));

    // Subscribe to sensor msgs
    fast_imu_sub_ = nh_->subscribe(imu_topic, 10, &OdomProvider::fastIMUCB, this);
    fast_dvl_sub_ = nh_->subscribe(dvl_topic, 10, &OdomProvider::fastDVLCB, this);
    tf_gt_subs_ = nh_->subscribe(gt_topic, 10, &OdomProvider::gtCB, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic, 10);

    // Initialize internal params
    init();

    // Main spin loop
    timer_ = nh_->createTimer(ros::Duration(1.0 / std::max(freq, 1.0)), &OdomProvider::provideOdom, this);

}

void OdomProvider::init(){

    // State machine
    init_filter_ = false;
    coord_ = false;

    // Initialize odometry
    cumul_odom_.setZero(6);

    // Masks sizes for interpolation of sensor inputs
    size_imu_q_ = 50;
    size_dvl_q_ = 10;

    // Get fixed transform dvl_link --> base_link frame
    tf::TransformListener tf_listener;
    try {
        tf_listener.waitForTransform(base_frame_, dvl_frame_, ros::Time(0), ros::Duration(10.0) );
        tf_listener.lookupTransform(base_frame_, dvl_frame_, ros::Time(0), transf_dvl_base_);
        ROS_INFO("Locked transform dvl --> base");
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
        ros::Duration(1.0).sleep();
    }

    // Get fixed transform world --> odom frame
    try {
        tf_listener.waitForTransform(world_frame_, odom_frame_, ros::Time(0), ros::Duration(10.0) );
        tf_listener.lookupTransform(world_frame_, odom_frame_, ros::Time(0), transf_world_odom_);
        ROS_INFO("Locked transform world --> odom");
        // Compute inverse for later use
        transf_odom_world_ = transf_world_odom_.inverse();
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
        ros::Duration(1.0).sleep();
    }

    ROS_INFO_NAMED(node_name_, "Initialized");
}


void OdomProvider::fastIMUCB(const sensor_msgs::Imu &imu_msg){
    imu_readings_.push_back(imu_msg);
    while(imu_readings_.size() > size_imu_q_){
        imu_readings_.pop_front();
    }
}

void OdomProvider::fastDVLCB(const geometry_msgs::TwistWithCovarianceStamped &dvl_msg){
    boost::mutex::scoped_lock lock(msg_lock_);
    dvl_readings_.push_back(dvl_msg);
    while(dvl_readings_.size() > size_dvl_q_){
        dvl_readings_.pop_front();
    }
}

void OdomProvider::synchSensorsCB(const sensor_msgs::ImuConstPtr &imu_msg,
                                  const geometry_msgs::TwistWithCovarianceStampedConstPtr &dvl_msg){
    coord_ = true;
}

void OdomProvider::gtCB(const nav_msgs::Odometry &pose_msg){
    gt_readings_.push_back(pose_msg);
    unsigned int size_gt_q = 10;
    while(gt_readings_.size() > size_gt_q){
        gt_readings_.pop_front();
    }
}


void OdomProvider::interpolateDVL(ros::Time t_now, geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg_ptr){

    geometry_msgs::Vector3 u_interp;
    u_interp.x = 0.0;
    u_interp.y = 0.0;
    u_interp.z = 0.0;

    // Lock to prevent concurrent access to dvl_readings_
    boost::mutex::scoped_lock lock(msg_lock_);
    unsigned int n_fac = 1;
    unsigned int n = dvl_readings_.size();
    double aux[n];
    n = n-1;
    n_fac = factorial(n);

    for(unsigned int l=0; l<=n; l++){
        aux[l] =  (n_fac / (factorial(l) * factorial(n - l))) *
                   std::pow(1 - (t_now.toSec() - dvl_readings_.at(n).header.stamp.toSec())/
                            (dvl_readings_.at(n).header.stamp.toSec() - dvl_readings_.at(n - n).header.stamp.toSec()), n-l) *
                   std::pow((t_now.toSec() - dvl_readings_.at(n).header.stamp.toSec())/
                            (dvl_readings_.at(n).header.stamp.toSec() - dvl_readings_.at(n - n).header.stamp.toSec()), l);
        u_interp.x += dvl_readings_.at(n - l).twist.twist.linear.x * aux[l];
        u_interp.y += dvl_readings_.at(n - l).twist.twist.linear.y * aux[l];
        u_interp.z += dvl_readings_.at(n - l).twist.twist.linear.z * aux[l];
    }

    // New interpolated reading
    dvl_msg_ptr.reset(new geometry_msgs::TwistWithCovarianceStamped{});
    dvl_msg_ptr->header.stamp = t_now;
    dvl_msg_ptr->twist.twist.linear = u_interp;
}

void OdomProvider::computeOdom(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg,
                               const tf::Quaternion& q_auv, Eigen::VectorXd &u_t){

    // Update time step
    double t_now = dvl_msg->header.stamp.toSec();
    double delta_t = t_now - t_prev_;

    // Transform from dvl input form dvl --> base_link frame
    tf::Vector3 twist_vel(dvl_msg->twist.twist.linear.x,
                          dvl_msg->twist.twist.linear.y,
                          dvl_msg->twist.twist.linear.z);
    tf::Vector3 disp_base = transf_dvl_base_.getBasis() * twist_vel * delta_t;

    // Compute increments in x,y,z in odom frame
    tf::Matrix3x3 rot_base_odom;
    rot_base_odom.setRotation(q_auv);
    tf::Vector3 disp_odom = rot_base_odom * disp_base;

    // Compute increments in roll,pitch,yaw in odom frame
    tfScalar pitch_t, roll_t, yaw_t;
    tf::Matrix3x3(q_auv).getRPY(roll_t, pitch_t, yaw_t);
    double droll = angleLimit(roll_t - cumul_odom_(3));
    double dpitch = angleLimit(pitch_t - cumul_odom_(4));
    double dtheta = angleLimit(yaw_t - cumul_odom_(5));

    // Incremental part of the motion model
    u_t(0) = disp_odom.x();
    u_t(1) = disp_odom.y();
    u_t(2) = disp_odom.z();
    u_t(3) = droll;
    u_t(4) = dpitch;
    u_t(5) = dtheta;

    cumul_odom_ += u_t;
    cumul_odom_(3) = angleLimit(cumul_odom_(3));
    cumul_odom_(4) = angleLimit(cumul_odom_(4));
    cumul_odom_(5) = angleLimit(cumul_odom_(5));

    t_prev_ = t_now;
}

bool OdomProvider::sendOutput(ros::Time t){

    tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(cumul_odom_(3), cumul_odom_(4), cumul_odom_(5));
    q_auv_t.normalize();
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q_auv_t, odom_quat);

    // Broadcast transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = t;
    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id = base_frame_;
    odom_trans.transform.translation.x = cumul_odom_(0);
    odom_trans.transform.translation.y = cumul_odom_(1);
    odom_trans.transform.translation.z = cumul_odom_(2);
    odom_trans.transform.rotation = odom_quat;
    odom_bc_.sendTransform(odom_trans);

    // Publish odom msg
    nav_msgs::Odometry odom_inertial_msg;
    odom_inertial_msg.header.stamp = t;
    odom_inertial_msg.header.frame_id = odom_frame_;
    odom_inertial_msg.child_frame_id = base_frame_;
    odom_inertial_msg.pose.pose.position.x = cumul_odom_(0);
    odom_inertial_msg.pose.pose.position.y = cumul_odom_(1);
    odom_inertial_msg.pose.pose.position.z = cumul_odom_(2);
    odom_inertial_msg.pose.pose.orientation = odom_quat;
    odom_pub_.publish(odom_inertial_msg);

    return true;
}

void OdomProvider::provideOdom(const ros::TimerEvent& e){

    sensor_msgs::ImuPtr imu_msg;
    geometry_msgs::TwistWithCovarianceStampedPtr dvl_msg;
    nav_msgs::OdometryPtr gt_msg;

    // TODO: predefine matrices so that they can be allocated in the stack!
    tf::Quaternion q_auv;
    Eigen::VectorXd u_t(6);

    if(dvl_readings_.size() >= size_dvl_q_ && imu_readings_.size() >= size_imu_q_ && !gt_readings_.empty()){
        // Init filter with initial, true pose (from GPS?)
        if(!init_filter_){
            ROS_INFO_NAMED(node_name_, "Starting navigation node");

            // Compute initial pose
            gt_msg = boost::make_shared<nav_msgs::Odometry>(gt_readings_.back());
            t_prev_ = gt_msg->header.stamp.toSec();

            // Transform IMU output world --> odom
            tf::Quaternion q_transf;
            tf::quaternionMsgToTF(gt_msg->pose.pose.orientation, q_transf);
            q_auv = transf_odom_world_.getRotation() * q_transf;
            q_auv.normalize();

            // Publish odom increment
            this->sendOutput(gt_msg->header.stamp);

            init_filter_ = true;
        }
        // MAIN LOOP
        else{
            // Fetch latest sensor readings
            imu_msg = boost::make_shared<sensor_msgs::Imu>(imu_readings_.back());

            if(coord_ == false){
                // IMU available but not DVL
                this->interpolateDVL(imu_msg->header.stamp, dvl_msg);
            }
            else{
                // Sensor input available on both channels
                coord_ = false;
                dvl_msg = boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>(dvl_readings_.back());
            }

            // Transform IMU output world --> odom
            tf::Quaternion q_transf;
            tf::quaternionMsgToTF(imu_msg->orientation, q_transf);
            q_auv = transf_odom_world_.getRotation() * q_transf;
            q_auv.normalize();

            // Compute displacement based on DVL and IMU orientation
            computeOdom(dvl_msg, q_auv, u_t);

            // Publish odom increment
            this->sendOutput(dvl_msg->header.stamp);
        }
    }
    else{
        gt_msg = boost::make_shared<nav_msgs::Odometry>(gt_readings_.back());
        this->sendOutput(gt_msg->header.stamp);
        ROS_WARN("No sensory update, broadcasting latest known pose");
    }

    // Empty the pointers
    imu_msg.reset();
    dvl_msg.reset();
    gt_msg.reset();

}

OdomProvider::~OdomProvider(){

}
