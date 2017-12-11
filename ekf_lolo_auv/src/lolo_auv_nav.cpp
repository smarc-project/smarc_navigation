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

    init();
}

void LoLoEKF::init(){
    mu_ = boost::numeric::ublas::zero_vector<double>(3);
    Sigma_ = boost::numeric::ublas::identity_matrix<double>(3);
    R_ = boost::numeric::ublas::identity_matrix<double> (3) * 0.001; // TODO: set diagonal as rosparam
    Q_ = boost::numeric::ublas::identity_matrix<double> (2) * 0.001;

}

double LoLoEKF::angleLimit (double angle) const{ // keep angle within [-pi;pi)
        return std::fmod(angle + M_PI, (M_PI * 2)) - M_PI;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
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

bool LoLoEKF::sendOutput(ros::Time &t){

    tf::Quaternion q_auv = tf::createQuaternionFromRPY(0, 0, mu_(2));
    q_auv.normalize();
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q_auv, odom_quat);

    // Broadcast transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = t;
    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id = base_frame_;
    odom_trans.transform.translation.x = mu_(0);
    odom_trans.transform.translation.y = mu_(1);
    odom_trans.transform.translation.z = z_t_;
    odom_trans.transform.rotation = odom_quat;
    odom_bc_.sendTransform(odom_trans);

    // Publish odom msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = t;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = mu_(0);
    odom_msg.pose.pose.position.y = mu_(1);
    odom_msg.pose.pose.position.z = z_t_;
    odom_msg.pose.pose.orientation = odom_quat;
    odom_pub_.publish(odom_msg);

    return true;
}

void LoLoEKF::computeOdom(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg, const tf::Quaternion q_auv,
                          double &t_prev, boost::numeric::ublas::vector<double> &u_t){

    // Update time step
    double t_now = dvl_msg->header.stamp.toSec();
    double delta_t = t_now - t_prev;

    // Transform from dvl input form dvl --> base_link frame
    tf::Vector3 twist_vel(dvl_msg->twist.twist.linear.x,
                          dvl_msg->twist.twist.linear.y,
                          dvl_msg->twist.twist.linear.z);
    tf::Vector3 l_vel_base = transf_dvl_base_.getBasis() * twist_vel;

    // Compute incremental displacements
    double vel_t = std::sqrt(pow((l_vel_base.y()),2) +
                            pow((l_vel_base.x()),2));

    double yaw_t = tf::getYaw(q_auv);
    double dtheta = angleLimit(yaw_t - mu_(2));
    double theta = angleLimit(mu_(2) + dtheta/2);

    // Compute control u_t
    int sign = (sgn(std::sin(theta) * vel_t) == 1)? -1: 1;
    u_t(0) = std::cos(theta) * vel_t * delta_t;
    u_t(1) = std::sin(theta) * vel_t * delta_t;
    u_t(2) = dtheta;

    // Derivative of motion model
    G_t_ = boost::numeric::ublas::identity_matrix<double>(3);
    G_t_(0,2) = -0.5 * vel_t * delta_t * std::sin(theta);
    G_t_(1,2) = 0.5 * vel_t * delta_t * std::cos(theta);
    G_t_(2,2) = 0;

    t_prev = t_now;
}

void LoLoEKF::prediction(boost::numeric::ublas::vector<double> &u_t){

    // Compute predicted mu
    mu_hat_ = mu_ + u_t;
    mu_hat_(2) = angleLimit(mu_hat_(2));

    // Predicted covariance matrix
    boost::numeric::ublas::matrix<double> aux = boost::numeric::ublas::prod(G_t_, Sigma_);
    Sigma_hat_ = boost::numeric::ublas::prod(aux, boost::numeric::ublas::trans(G_t_));
    Sigma_hat_ += R_;
}

void LoLoEKF::update(){
    mu_ = mu_hat_;
}

void LoLoEKF::transIMUframe(const geometry_msgs::Quaternion &auv_quat, tf::Quaternion &q_auv){
    // Transform IMU orientation from world to odom coordinates
    tf::Quaternion q_transf;
    tf::quaternionMsgToTF(auv_quat, q_transf);
    tf::Quaternion q_world_odom = transf_world_odom_.getRotation();
    q_auv = q_world_odom * q_transf;
    q_auv.normalize();  // TODO: implement handling of singularities?
}

void LoLoEKF::ekfLocalize(){
    ros::Rate rate(10);

    sensor_msgs::ImuPtr imu_msg;
    geometry_msgs::TwistWithCovarianceStampedPtr dvl_msg;
    nav_msgs::OdometryPtr gt_msg;
    double t_prev;
    tf::Quaternion q_auv;

    // TODO: full implementation of 6 DOF movement

    boost::numeric::ublas::vector<double> u_t = boost::numeric::ublas::vector<double>(3);

    bool loop = true;
    bool init_filter = true;

    while(ros::ok()&& loop){
        ros::spinOnce();
        if(!dvl_readings_.empty() && !imu_readings_.empty() && !gt_readings_.empty()){

            // Init filter with initial, true pose (from GPS?)
            if(init_filter){
                ROS_INFO("Starting localization node");

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
                }
                catch(tf::TransformException &exception) {
                    ROS_ERROR("%s", exception.what());
                    ros::Duration(1.0).sleep();
                }

                // Compute initial pose
                gt_msg = gt_readings_.back();
                t_prev = gt_msg->header.stamp.toSec();
                transIMUframe(gt_msg->pose.pose.orientation, q_auv);

                // Publish and broadcast
                z_t_ = gt_msg->pose.pose.position.z - transf_world_odom_.getOrigin().getZ(); // Imitate depth sensor input
                this->sendOutput(gt_msg->header.stamp);

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
            transIMUframe(imu_msg->orientation, q_auv);
            computeOdom(dvl_msg, q_auv, t_prev, u_t);

            // Prediction step
            prediction(u_t);
            z_t_ = gt_msg->pose.pose.position.z - transf_world_odom_.getOrigin().getZ(); // Imitate depth sensor input

            // Update step
            update();

//            double error_x = std::sqrt(pow(gt_msg->pose.pose.position.x - mu_(0), 2));
//            double error_y = std::sqrt(pow(gt_msg->pose.pose.position.y - mu_(1), 2));
//            std::cout << "Error in x: " << error_x << " and y: " << error_y << std::endl;

            // Publish and broadcast
            this->sendOutput(dvl_msg->header.stamp);

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
