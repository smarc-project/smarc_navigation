#include "ekf_slam/ekf_slam.hpp"

// HELPER FUNCTIONS TODO: move to aux library
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// END HELPER FUNCTIONS


EKFSLAM::EKFSLAM(std::string node_name, ros::NodeHandle &nh): nh_(&nh), node_name_(node_name){

    std::string map_topic;
    std::string odom_topic;
    std::string observs_topic;
    double freq;
    double delta;
    std::vector<double> R_diagonal;
    std::vector<double> Q_diagonal;
    std::vector<double> Sigma_diagonal;

    nh_->param("init_pose_cov_diag", Sigma_diagonal, std::vector<double>());
    nh_->param("motion_noise_cov_diag", R_diagonal, std::vector<double>());
    nh_->param("meas_noise_cov_diag", Q_diagonal, std::vector<double>());
    nh_->param<double>((node_name_ + "/delta_outlier_reject"), delta, 0.99);
    nh_->param<double>((node_name_ + "/system_freq"), freq, 30);
    nh_->param<std::string>((node_name_ + "/map_pose_topic"), map_topic, "/map_ekf");
    nh_->param<std::string>((node_name_ + "/odom_pub_topic"), odom_topic, "/odom_ekf");
    nh_->param<std::string>((node_name_ + "/lm_detect_topic"), observs_topic, "/landmarks_detected");
    nh_->param<std::string>((node_name_ + "/world_frame"), world_frame_, "/world");
    nh_->param<std::string>((node_name_ + "/map_frame"), map_frame_, "/map");
    nh_->param<std::string>((node_name_ + "/odom_frame"), odom_frame_, "/odom");
    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");

    // Subscribe to sensor msgs
    observs_subs_ = nh_->subscribe(observs_topic, 10, &EKFSLAM::observationsCB, this);
    odom_subs_ = nh_->subscribe(odom_topic, 10, &EKFSLAM::odomCB, this);
    map_pub_ = nh_->advertise<nav_msgs::Odometry>(map_topic, 10);

    // Plot map in RVIZ
    vis_pub_ = nh_->advertise<visualization_msgs::MarkerArray>( "/lolo_auv/rviz/landmarks", 0 );

    // Initialize internal params
    init(Sigma_diagonal, R_diagonal, Q_diagonal, delta);

    // Main spin loop
    timer_ = nh_->createTimer(ros::Duration(1.0 / std::max(freq, 1.0)), &EKFSLAM::ekfLocalize, this);

}

void EKFSLAM::init(std::vector<double> sigma_diag, std::vector<double> r_diag, std::vector<double> q_diag, double delta){

    // EKF variables
    double size_state = r_diag.size();
    double size_meas = q_diag.size();
    mu_.setZero(size_state);
    Sigma_ = Eigen::MatrixXd::Identity(size_state, size_state);

    for(unsigned int i=0; i<size_state; i++){
        Sigma_(i,i) = sigma_diag.at(i);
    }
    R_ = Eigen::MatrixXd::Identity(size_state, size_state);
    for(unsigned int i=0; i<size_state; i++){
        R_(i,i) = r_diag.at(i);
    }
    Q_ = Eigen::MatrixXd::Identity(size_meas, size_meas);
    for(unsigned int i=0; i<size_meas; i++){
        Q_(i,i) = q_diag.at(i);
    }

    // Outlier rejection
    boost::math::chi_squared chi2_dist(size_meas);
    lambda_M_ = boost::math::quantile(chi2_dist, delta);

    // Aux
    size_odom_q_ = 10;

    // Create EKF filter
    ekf_filter_ = new EKFCore(mu_, Sigma_, R_, Q_, lambda_M_);

    ROS_INFO_NAMED(node_name_, "EKF SLAM Initialized");
}

void EKFSLAM::odomCB(const nav_msgs::Odometry &odom_msg){
    odom_queue_t_.push_back(odom_msg);
    while(odom_queue_t_.size() > size_odom_q_){
        odom_queue_t_.pop_front();
    }
}

void EKFSLAM::observationsCB(const geometry_msgs::PoseArray &observ_msg){
    measurements_t_.push_back(observ_msg);
}


void EKFSLAM::updateMapMarkers(double color){

    visualization_msgs::MarkerArray marker_array;
    Eigen::Vector3d landmark;
    for(unsigned int j=0; j<(mu_.rows()-6)/3; j++){
        landmark = mu_.segment(3 * j + 6, 3);
        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = ros::Time();
        marker.ns = "map_array";
        marker.id = j;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = landmark(0);
        marker.pose.position.y = landmark(1);
        marker.pose.position.z = landmark(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = color;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_array.markers.push_back(marker);
    }
    vis_pub_.publish(marker_array);
}

bool EKFSLAM::sendOutput(ros::Time t){

    // Publish odom filtered msg
    tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(mu_(3), mu_(4), mu_(5)).normalize();
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q_auv_t, odom_quat);

    nav_msgs::Odometry odom_filtered_msg;
    odom_filtered_msg.header.stamp = t;
    odom_filtered_msg.header.frame_id = map_frame_;
    odom_filtered_msg.child_frame_id = base_frame_;
    odom_filtered_msg.pose.pose.position.x = mu_(0);
    odom_filtered_msg.pose.pose.position.y = mu_(1);
    odom_filtered_msg.pose.pose.position.z = mu_(2);
    odom_filtered_msg.pose.pose.orientation = odom_quat;
    map_pub_.publish(odom_filtered_msg);

    return true;
}

bool EKFSLAM::bcMapOdomTF(ros::Time t){
    // Get transform odom --> base published by odom_provider
    tf::StampedTransform tf_base_odom;
    try {
        tf_listener_.waitForTransform(base_frame_, odom_frame_, t, ros::Duration(0.1));
        tf_listener_.lookupTransform(base_frame_, odom_frame_, t, tf_base_odom);
        // Build tf map --> base from estimated pose
        tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(mu_(3), mu_(4), mu_(5)).normalize();
        tf::Transform tf_base_map = tf::Transform(q_auv_t, tf::Vector3(mu_(0), mu_(1), mu_(2)));

        // Compute map --> odom transform
        tf::Transform tf_map_odom;
        tf_map_odom.mult(tf_base_map, tf_base_odom);
        tf::StampedTransform tf_odom_map_stp = tf::StampedTransform(tf_map_odom,
                                               ros::Time::now(),
                                               map_frame_,
                                               odom_frame_);

        // Broadcast map --> odom transform
        geometry_msgs::TransformStamped msg_odom_map_;
        tf::transformStampedTFToMsg(tf_odom_map_stp, msg_odom_map_);
        map_bc_.sendTransform(msg_odom_map_);
    }
    catch(tf::TransformException &exception) {
        ROS_WARN("%s", exception.what());
        ROS_ERROR("Skipping map --> odom broadcasting iteration");
    }
}

void EKFSLAM::ekfLocalize(const ros::TimerEvent& e){

    // TODO: predefine matrices so that they can be allocated in the stack!
    nav_msgs::Odometry odom_reading;
    std::vector<Eigen::Vector3d> z_t;

    if(!odom_queue_t_.empty()){
        // Fetch latest measurement
        odom_reading = odom_queue_t_.back();
        odom_queue_t_.pop_back();

        // Prediction step
        ekf_filter_->predictMotion(odom_reading);

        // If observations available:
        if(!measurements_t_.empty()){
            // Fetch latest measurement
            auto observ = measurements_t_.back();
            measurements_t_.pop_back();
            if(!measurements_t_.empty()){
                ROS_WARN("Cache with measurements is not empty");
            }

            for(auto lm_pose: observ.poses){
                z_t.push_back(Eigen::Vector3d(lm_pose.position.x,
                                              lm_pose.position.y,
                                              lm_pose.position.z));
            }

            // Data association and sequential update
            ekf_filter_->dataAssociation(z_t);
            z_t.clear();
        }

        // Update mu_ and Sigma_
        std::tie(mu_, Sigma_) = ekf_filter_->ekfUpdate();

        // Publish and broadcast
        this->sendOutput(ros::Time::now());
        this->bcMapOdomTF(ros::Time::now());
        this->updateMapMarkers(1.0);
    }
    else{
        ROS_WARN("No odometry info received, bc identity map --> odom transform");
        this->sendOutput(ros::Time::now());

        // Build tf map --> base from latest update or identity
        tf::StampedTransform tf_odom_map_stp;
        tf::Transform tf_map_odom = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
        tf_odom_map_stp = tf::StampedTransform(tf_map_odom,
                                               ros::Time::now(),
                                               map_frame_,
                                               odom_frame_);
        // Broadcast map --> odom transform
        map_bc_.sendTransform(tf_odom_map_stp);
    }
}

EKFSLAM::~EKFSLAM(){
    // Empty queues
    measurements_t_.clear();
    odom_queue_t_.clear();

    // Delete instance pointers
    delete(nh_);
    delete(ekf_filter_);
}
