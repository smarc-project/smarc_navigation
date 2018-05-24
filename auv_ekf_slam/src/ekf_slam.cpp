#include "ekf_slam/ekf_slam.hpp"

EKFSLAM::EKFSLAM(std::string node_name, ros::NodeHandle &nh): nh_(&nh), node_name_(node_name){

    std::string map_topic;
    std::string odom_topic;
    std::string observs_topic;
    std::string lm_topic;
    double freq;
    double delta;
    double mh_dist_mbes;
    double mh_dist_fls;
    std::vector<double> R_diagonal;
    std::vector<double> Q_fls_diag;
    std::vector<double> Q_mbes_diag;
    std::vector<double> Sigma_diagonal;

    nh_->param("init_pose_cov_diag", Sigma_diagonal, std::vector<double>());
    nh_->param("motion_noise_cov_diag", R_diagonal, std::vector<double>());
    nh_->param("meas_fls_noise_cov_diag", Q_fls_diag, std::vector<double>());
    nh_->param("meas_mbes_noise_cov_diag", Q_mbes_diag, std::vector<double>());
    nh_->param<double>((node_name_ + "/delta_outlier_reject"), delta, 0.99);
    nh_->param<double>((node_name_ + "/system_freq"), freq, 30);
    nh_->param<double>((node_name_ + "/mhl_dist_fls"), mh_dist_fls, 0.5);
    nh_->param<double>((node_name_ + "/mhl_dist_mbes"), mh_dist_mbes, 0.2);
    nh_->param<std::string>((node_name_ + "/map_pose_topic"), map_topic, "/map_ekf");
    nh_->param<std::string>((node_name_ + "/odom_pub_topic"), odom_topic, "/odom_ekf");
    nh_->param<std::string>((node_name_ + "/lm_detect_topic"), observs_topic, "/landmarks_detected");
    nh_->param<std::string>((node_name_ + "/world_frame"), world_frame_, "/world");
    nh_->param<std::string>((node_name_ + "/fls_frame"), fls_frame_, "/forward_sonardown_link");
    nh_->param<std::string>((node_name_ + "/mbes_frame"), mbes_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/map_frame"), map_frame_, "/map");
    nh_->param<std::string>((node_name_ + "/odom_frame"), odom_frame_, "/odom");
    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/landmarks_adv"), lm_topic, "/lolo_auv/rviz/landmarks");
    nh_->param<std::string>((node_name_ + "/map_srv"), map_srv_, "/lolo_auv/map_server");

    // Subscribe to sensor msgs
    observs_subs_ = nh_->subscribe(observs_topic, 10, &EKFSLAM::observationsCB, this);
    odom_subs_ = nh_->subscribe(odom_topic, 10, &EKFSLAM::odomCB, this);
    map_pub_ = nh_->advertise<nav_msgs::Odometry>(map_topic, 10);

    // Plot map in RVIZ
    vis_pub_ = nh_->advertise<visualization_msgs::MarkerArray>(lm_topic, 0 );

    // Get initial map of beacons from Gazebo
    init_map_client_ = nh_->serviceClient<smarc_lm_visualizer::init_map>(map_srv_);

    // Initialize internal params
    init(Sigma_diagonal, R_diagonal, Q_fls_diag, Q_mbes_diag, delta, mh_dist_fls, mh_dist_mbes);

    // Main spin loop
    timer_ = nh_->createTimer(ros::Duration(1.0 / std::max(freq, 1.0)), &EKFSLAM::ekfLocalize, this);


}

void EKFSLAM::init(std::vector<double> sigma_diag, std::vector<double> r_diag, std::vector<double> q_fls_diag, std::vector<double> q_mbes_diag, double delta, double mhl_dist_fls, double mhl_dist_mbes){

    // Init EKF variables
    double size_state = r_diag.size();
    double size_meas_fls = q_fls_diag.size();
    double size_meas_mbes = q_mbes_diag.size();
    mu_.setZero(size_state);
    Sigma_ = Eigen::MatrixXd::Identity(size_state, size_state);

    for(unsigned int i=0; i<size_state; i++){
        Sigma_(i,i) = sigma_diag.at(i);
    }
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(size_state, size_state);
    for(unsigned int i=0; i<size_state; i++){
        R(i,i) = r_diag.at(i);
    }


    Eigen::MatrixXd Q_fls = Eigen::MatrixXd::Identity(size_meas_fls, size_meas_fls);
    for(unsigned int i=0; i<size_meas_fls; i++){
        Q_fls(i,i) = q_fls_diag.at(i);
    }

    Eigen::MatrixXd Q_mbes = Eigen::MatrixXd::Identity(size_meas_mbes, size_meas_mbes);
    for(unsigned int i=0; i<size_meas_mbes; i++){
        Q_mbes(i,i) = q_mbes_diag.at(i);
    }

    // Outlier rejection
    boost::math::chi_squared chi2_dist(size_meas_fls);
    double lambda_fls = boost::math::quantile(chi2_dist, delta);
    boost::math::chi_squared chi3_dist(size_meas_mbes);
    double lambda_mbes = boost::math::quantile(chi3_dist, delta);

    // Aux
    size_odom_q_ = 10;
    size_measurements_q_ = 1;

    // Listen to necessary, fixed tf transforms
    tf::StampedTransform tf_base_sensor;
    try {
        tf_listener_.waitForTransform(base_frame_, fls_frame_, ros::Time(0), ros::Duration(100));
        tf_listener_.lookupTransform(base_frame_, fls_frame_, ros::Time(0), tf_base_sensor);
        ROS_INFO_STREAM(node_name_ << ": locked transform sensor --> frame");
    }
    catch(tf::TransformException &exception) {
        ROS_WARN("%s", exception.what());
    }

    try {
        tf_listener_.waitForTransform(map_frame_, world_frame_, ros::Time(0), ros::Duration(100));
        tf_listener_.lookupTransform(map_frame_, world_frame_, ros::Time(0), transf_map_world_);
        ROS_INFO_STREAM(node_name_ << ": locked transform world --> map");
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
        ros::Duration(1.0).sleep();
    }

    // Initialize tf map --> base (identity)
    tf::StampedTransform tf_odom_map_stp;
    tf::Transform tf_map_odom = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
    tf_odom_map_stp = tf::StampedTransform(tf_map_odom,
                                           ros::Time::now(),
                                           map_frame_,
                                           odom_frame_);
    tf::transformStampedTFToMsg(tf_odom_map_stp, msg_odom_map_);


    // Initial map of the survey area (usually artificial beacons)
    while(!ros::service::waitForService(map_srv_, ros::Duration(10)) && ros::ok()){
        ROS_INFO_STREAM(node_name_ << ": waiting for the map server service to come up");
    }

    smarc_lm_visualizer::init_map init_map_srv;
    init_map_srv.request.request_map = true;

    init_map_client_.call(init_map_srv);

    if(!init_map_srv.response.init_map.poses.empty()){
        Eigen::VectorXd aux_mu;
        tf::Vector3 beacon_pose;
        // Add beacon landmarks to mu_
        for(auto beacon: init_map_srv.response.init_map.poses){
            // Transform beacon from world to odom frame
            beacon_pose = transf_map_world_ * tf::Vector3(beacon.position.x,
                                                           beacon.position.y,
                                                           beacon.position.z);
            // Augment mu_
            aux_mu = mu_;
            mu_.conservativeResize(mu_.size()+3, true);

            mu_ << aux_mu, Eigen::Vector3d(beacon_pose.getX(),
                                           beacon_pose.getY(),
                                           beacon_pose.getZ());

            // Augment Sigma_
            Sigma_.conservativeResize(Sigma_.rows()+3, Sigma_.cols()+3);
            Sigma_.bottomRows(3).setZero();
            Sigma_.rightCols(3).setZero();
            Sigma_(Sigma_.rows()-3, Sigma_.cols()-3) = 20;
            Sigma_(Sigma_.rows()-2, Sigma_.cols()-2) = 10;
            Sigma_(Sigma_.rows()-1, Sigma_.cols()-1) = 10;
        }
    }
    std::cout << "Initial mu: " << mu_.size() << std::endl;
    std::cout << "Initial Sigma: " << Sigma_.cols() << std::endl;
    std::cout << "Number of landmarks: " << (Sigma_.rows() - 6) / 3 << std::endl;

    // Create EKF filter
    ekf_filter_ = new EKFCore(mu_, Sigma_, R, Q_fls, Q_mbes, lambda_fls, lambda_mbes, tf_base_sensor, mhl_dist_fls, mhl_dist_mbes);

    ROS_INFO_STREAM(node_name_  << ": initialized");
}

void EKFSLAM::odomCB(const nav_msgs::Odometry &odom_msg){
    odom_queue_t_.push_back(odom_msg);
    while(odom_queue_t_.size() > size_odom_q_){
        odom_queue_t_.pop_front();
    }
}

void EKFSLAM::observationsCB(const geometry_msgs::PoseArray &observ_msg){
    measurements_t_.push_back(observ_msg);
    while(measurements_t_.size() > size_measurements_q_){
         measurements_t_.pop_front();
    }
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
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = landmark(0);
        marker.pose.position.y = landmark(1);
        marker.pose.position.z = landmark(2);
        marker.pose.orientation.x = 1.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = color;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.mesh_resource = "package://smarc_worlds/world_models/large_rock/meshes/large_rock.dae";

        marker_array.markers.push_back(marker);
    }
    vis_pub_.publish(marker_array);
}

bool EKFSLAM::sendOutput(ros::Time t_meas){

    // Publish odom filtered msg
    tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(mu_(3), mu_(4), mu_(5)).normalize();
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q_auv_t, odom_quat);

    nav_msgs::Odometry odom_filtered_msg;
    odom_filtered_msg.header.stamp = t_meas;
    odom_filtered_msg.header.frame_id = map_frame_;
    odom_filtered_msg.child_frame_id = base_frame_;
    odom_filtered_msg.pose.pose.position.x = mu_(0);
    odom_filtered_msg.pose.pose.position.y = mu_(1);
    odom_filtered_msg.pose.pose.position.z = mu_(2);
    odom_filtered_msg.pose.pose.orientation = odom_quat;
    map_pub_.publish(odom_filtered_msg);

    return true;
}

bool EKFSLAM::bcMapOdomTF(ros::Time t_meas){
    // Get transform odom --> base published by odom_provider
    tf::StampedTransform tf_base_odom;
    bool broadcasted = false;
    try {
        tf_listener_.waitForTransform(base_frame_, odom_frame_, t_meas, ros::Duration(0.1));
        tf_listener_.lookupTransform(base_frame_, odom_frame_, t_meas, tf_base_odom);
        // Build tf map --> base from estimated pose
        tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(mu_(3), mu_(4), mu_(5)).normalize();
        tf::Transform tf_base_map = tf::Transform(q_auv_t, tf::Vector3(mu_(0), mu_(1), mu_(2)));

        // Compute map --> odom transform
        tf::Transform tf_map_odom;
        tf_map_odom.mult(tf_base_map, tf_base_odom);
        tf::StampedTransform tf_odom_map_stp = tf::StampedTransform(tf_map_odom,
                                               t_meas,
                                               map_frame_,
                                               odom_frame_);

        // Broadcast map --> odom transform
        tf_odom_map_stp.getRotation().normalize();
        tf::transformStampedTFToMsg(tf_odom_map_stp, msg_odom_map_);
        map_bc_.sendTransform(msg_odom_map_);
        broadcasted = true;
    }
    catch(tf::TransformException &exception) {
        ROS_WARN("%s", exception.what());
        ROS_ERROR("Skipping map --> odom broadcasting iteration");
    }

    return broadcasted;
}

void EKFSLAM::ekfLocalize(const ros::TimerEvent&){

    // TODO: predefine matrices so that they can be allocated in the stack!
    nav_msgs::Odometry odom_reading;
    std::vector<Eigen::Vector3d> z_t;
    utils::MeasSensor sensor_input;
    ros::Time t_meas;

    if(!odom_queue_t_.empty()){
        // Fetch latest measurement
//        ROS_INFO("------New odom info received------");
        odom_reading = odom_queue_t_.back();
        t_meas = odom_reading.header.stamp;
        odom_queue_t_.pop_back();

        // Prediction step
        ekf_filter_->predictMotion(odom_reading);

        // If observations available:
        if(!measurements_t_.empty()){
//            ROS_INFO("----New measurements received----");
            // Fetch latest measurement
            auto observ = measurements_t_.back();
            t_meas = observ.header.stamp;
            measurements_t_.pop_back();

            // Check the sensor type
            sensor_input = (observ.header.frame_id == mbes_frame_)? utils::MeasSensor::MBES: utils::MeasSensor::FLS;    // TODO: generalize condition statement
            for(auto lm_pose: observ.poses){
                z_t.push_back(Eigen::Vector3d(lm_pose.position.x,
                                              lm_pose.position.y,
                                              lm_pose.position.z));
            }

            // Data association and sequential update
            ekf_filter_->batchDataAssociation(z_t, sensor_input);
            z_t.clear();
        }

        // Update mu_ and Sigma_
        std::tie(mu_, Sigma_) = ekf_filter_->ekfUpdate();

        // Publish and broadcast
        this->sendOutput(t_meas);
        this->bcMapOdomTF(t_meas);
        this->updateMapMarkers(1.0);
    }
    else{
        ROS_WARN_STREAM(node_name_ << ": No odometry info received, bc latest known map --> odom tf");
        this->sendOutput(ros::Time::now());

        // Broadcast latest known map --> odom transform if no inputs received
        map_bc_.sendTransform(msg_odom_map_);
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
