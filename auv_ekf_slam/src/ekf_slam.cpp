#include "ekf_slam/ekf_slam.hpp"

EKFSLAM::EKFSLAM(std::string node_name, ros::NodeHandle &nh): nh_(&nh), node_name_(node_name){

    std::string map_topic;
    std::string odom_topic;
    std::string observs_topic;
    std::string cam_path_topic;
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
    nh_->param<std::string>((node_name_ + "/cam_pipe_topic"), cam_path_topic, "/lolo_auv/camera/path");
    nh_->param<std::string>((node_name_ + "/world_frame"), world_frame_, "/world");
    nh_->param<std::string>((node_name_ + "/fls_frame"), fls_frame_, "/forward_sonardown_link");
    nh_->param<std::string>((node_name_ + "/mbes_frame"), mbes_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/map_frame"), map_frame_, "/map");
    nh_->param<std::string>((node_name_ + "/odom_frame"), odom_frame_, "/odom");
    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");

    // Subscribe to sensor msgs
    observs_subs_ = nh_->subscribe(observs_topic, 10, &EKFSLAM::observationsCB, this);
    odom_subs_ = nh_->subscribe(odom_topic, 10, &EKFSLAM::odomCB, this);
    map_pub_ = nh_->advertise<nav_msgs::Odometry>(map_topic, 10);
    cam_subs_ = nh_->subscribe(cam_path_topic, 2, &EKFSLAM::camCB, this);

    // Plot map in RVIZ
    vis_pub_ = nh_->advertise<visualization_msgs::MarkerArray>( "/lolo_auv/rviz/landmarks", 0 );
    pipe_pub_ = nh_->advertise<nav_msgs::Path>("/lolo_auv/path_pipeline", 0);
    pipe_lm_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/lolo_auv/pipeline_landmark", 0);

    // Get initial map of beacons from Gazebo
    init_map_client_ = nh_->serviceClient<landmark_visualizer::init_map>("/lolo_auv/map_server");

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
    pipe_meas_cnt_ = 0;
    first_pipe_rcv_ = false;
    pipeline_mask_.header.frame_id = base_frame_;

    // Listen to necessary, fixed tf transforms
    tf::StampedTransform tf_base_sensor;
    try {
        tf_listener_.waitForTransform(base_frame_, fls_frame_, ros::Time(0), ros::Duration(10));
        tf_listener_.lookupTransform(base_frame_, fls_frame_, ros::Time(0), tf_base_sensor);
        ROS_INFO("Locked transform sensor --> frame");
    }
    catch(tf::TransformException &exception) {
        ROS_WARN("%s", exception.what());
    }

    try {
        tf_listener_.waitForTransform(map_frame_, world_frame_, ros::Time(0), ros::Duration(10));
        tf_listener_.lookupTransform(map_frame_, world_frame_, ros::Time(0), transf_map_world_);
        ROS_INFO("Locked transform world --> map");
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
    while(!ros::service::waitForService("/lolo_auv/map_server", ros::Duration(10)) && ros::ok()){
        ROS_INFO_NAMED(node_name_,"Waiting for the map server service to come up");
    }

    landmark_visualizer::init_map init_map_srv;
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
            Sigma_(Sigma_.rows()-3, Sigma_.cols()-3) = 10;
            Sigma_(Sigma_.rows()-2, Sigma_.cols()-2) = 10;
            Sigma_(Sigma_.rows()-1, Sigma_.cols()-1) = 10;
        }
    }
    std::cout << "Initial mu: " << mu_.size() << std::endl;
    std::cout << "Initial Sigma: " << Sigma_.cols() << std::endl;
    std::cout << "Number of landmarks: " << (Sigma_.rows() - 6) / 3 << std::endl;

    // Create EKF filter
    ekf_filter_ = new EKFCore(mu_, Sigma_, R, Q_fls, Q_mbes, lambda_fls, lambda_mbes, tf_base_sensor, mhl_dist_fls, mhl_dist_mbes);

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
    while(measurements_t_.size() > size_measurements_q_){
         measurements_t_.pop_front();
    }
}

void EKFSLAM::camCB(const nav_msgs::Path &pipe_path){
    if(!first_pipe_rcv_){
        first_pipe_rcv_ = true;
    }
    pipe_path_queue_t_.push_back(pipe_path);
    while(pipe_path_queue_t_.size() >= 2){
         pipe_path_queue_t_.pop_front();
    }
}

void EKFSLAM::updateMapMarkers(double color){

    visualization_msgs::MarkerArray marker_array;
    Eigen::Vector3d landmark;
    visualization_msgs::Marker marker;
    for(unsigned int j=0; j<(mu_.rows()-6)/3; j++){
        landmark = mu_.segment(3 * j + 6, 3);
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

    tf::Quaternion q = tf::createQuaternionFromRPY(mu_(3), mu_(4), mu_(5)).normalize();

    int cnt = (mu_.rows()-6)/3;
    for(Eigen::Vector3d& pipe_lm: map_pipe_){
        marker.header.frame_id = map_frame_;
        marker.header.stamp = ros::Time();
        marker.ns = "map_array";
        marker.id = cnt;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pipe_lm(0);
        marker.pose.position.y = pipe_lm(1);
        marker.pose.position.z = pipe_lm(2);
        marker.pose.orientation.x =  0.7071;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z =  0.7071;
        marker.pose.orientation.w = 0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        cnt +=1;
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
    bool broadcasted = false;
    try {
        tf_listener_.waitForTransform(base_frame_, odom_frame_, ros::Time(0), ros::Duration(0.1));
        tf_listener_.lookupTransform(base_frame_, odom_frame_, ros::Time(0), tf_base_odom);
        // Build tf map --> base from estimated pose
        tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(mu_(3), mu_(4), mu_(5)).normalize();
        tf::Transform tf_map_base = tf::Transform(q_auv_t, tf::Vector3(mu_(0), mu_(1), mu_(2)));

        // Compute map --> odom transform
        tf::Transform tf_map_odom;
        tf_map_odom.mult(tf_map_base, tf_base_odom);
//        tf_map_odom = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
        tf::StampedTransform tf_odom_map_stp = tf::StampedTransform(tf_map_odom,
                                               ros::Time::now(),
                                               map_frame_,
                                               odom_frame_);

        // Broadcast map --> odom transform
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

struct ThirdDegreePolyResidual {
  ThirdDegreePolyResidual(double x, double y): x_(x), y_(y) {}

  template <typename T> bool operator()(/*const T* const m3,*/
                                        const T* const m2,
                                        const T* const m1,
                                        const T* const c,
                                        T* residual) const {
    residual[0] = y_ - (/*m3[0] * pow(x_,3)*/ + m2[0] * pow(x_,2) + m1[0] * x_ + c[0]);
    return true;
  }

 private:
  const double x_;
  const double y_;
};

void EKFSLAM::computePipePath(const nav_msgs::Path &pipe_proj, geometry_msgs::PoseStamped& pipeline_lm){

    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;
    using ceres::CauchyLoss;
    using ceres::Problem;
    using ceres::Solve;
    using ceres::Solver;

    double m3 = 0.0;
    double m2 = 0.0;
    double m1 = 0.0;
    double c = 0.0;
    Problem problem;

    tf::Transform transf_base_map = tf::Transform(tf::createQuaternionFromRPY(mu_(3), mu_(4), mu_(5)).normalize(),
                                                  tf::Vector3(mu_(0), mu_(1), mu_(2))).inverse();

    tf::Vector3 pose_in_base;
    for (geometry_msgs::PoseStamped pose_sample: pipe_proj.poses) {
        pose_in_base = transf_base_map  * tf::Vector3(pose_sample.pose.position.x,
                                                      pose_sample.pose.position.y,
                                                      pose_sample.pose.position.z);
        // Store pose in base_frame
        pose_sample.header.frame_id = base_frame_;
        pose_sample.pose.position.x = pose_in_base.getX();
        pose_sample.pose.position.y = pose_in_base.getY();
        pose_sample.pose.position.z = pose_in_base.getZ();

        // Add it to the mask
        pipeline_mask_.poses.push_back(pose_sample);

        CostFunction* cost_function = new AutoDiffCostFunction<ThirdDegreePolyResidual, 1, /*1,*/ 1, 1, 1>(new ThirdDegreePolyResidual(pose_sample.pose.position.x, pose_sample.pose.position.y));

        problem.AddResidualBlock(cost_function, new CauchyLoss(0.5), /*&m3,*/ &m2, &m1, &c);
    }



    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    // Publish interpolated curve
    nav_msgs::Path pipeline_path;
    pipeline_path.header.frame_id = base_frame_;
    pipeline_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped new_pose;
    new_pose.header.frame_id = base_frame_;
    new_pose.header.stamp = ros::Time::now();

    for (const geometry_msgs::PoseStamped& pose_sample: pipeline_mask_.poses) {
        new_pose.pose.position.x = pose_sample.pose.position.x;
        new_pose.pose.position.y = /*m3*pow(pose_sample.pose.position.x,3)*/ + m2*pow(pose_sample.pose.position.x,2) + m1*pose_sample.pose.position.x + c;
        new_pose.pose.position.z = pose_sample.pose.position.z;

        pipeline_path.poses.push_back(new_pose);

    }
    pipe_pub_.publish(pipeline_path);

    // Solve poly for x=0 and publish for visualization
    pipeline_lm.header.frame_id = base_frame_;
    pipeline_lm.header.stamp = ros::Time::now();
    pipeline_lm.pose.position.x = 0.0;
    pipeline_lm.pose.position.y = /*m3*pow(smallest_x - i,3)*/ + m2*pow(0.0,2) + m1*(0.0) + c;
    pipeline_lm.pose.position.z = pipeline_mask_.poses.at(pipeline_mask_.poses.size()-1).pose.position.z;

    pipe_lm_pub_.publish(pipeline_lm);

    pipeline_mask_.poses.clear();

}

void EKFSLAM::ekfLocalize(const ros::TimerEvent&){

    // TODO: predefine matrices so that they can be allocated in the stack!
    nav_msgs::Odometry odom_reading;
    nav_msgs::Path pipe_path_t;
    std::vector<Eigen::Vector3d> z_t;
    utils::MeasSensor sensor_input;
    geometry_msgs::PoseStamped pipeline_lm;

    if(!odom_queue_t_.empty()){
        // Fetch latest measurement
        ROS_INFO("------New odom info received------");
        odom_reading = odom_queue_t_.back();
        odom_queue_t_.pop_back();

        // Prediction step
        ekf_filter_->predictMotion(odom_reading);

        // Pipe detected by camera (1 Hz freq)
        if(first_pipe_rcv_){
            pipe_meas_cnt_ = 0;
            if(!pipe_path_queue_t_.empty()){
                pipe_path_t = pipe_path_queue_t_.back();
                pipe_path_queue_t_.pop_back();

                // If the camera has found the pipe
                if(!pipe_path_t.poses.empty()){
//                    updatePipelineMask(pipe_path_t);
                    computePipePath(pipe_path_t, pipeline_lm);
                    // If observations available:
                    if(!measurements_t_.empty()){
                        ROS_INFO("----New measurements received----");
                        // Fetch latest measurement
                        auto observ = measurements_t_.back();
                        measurements_t_.pop_back();

                        // Check the sensor type
                        sensor_input = (observ.header.frame_id == mbes_frame_)? utils::MeasSensor::MBES: utils::MeasSensor::FLS;    // TODO: generalize condition statement
                        for(auto lm_pose: observ.poses){
                            z_t.push_back(Eigen::Vector3d(lm_pose.position.x,
                                                          lm_pose.position.y,
                                                          lm_pose.position.z));
                        }

                        // Data association and sequential update
                        ekf_filter_->dataAssociation(z_t, sensor_input, pipeline_lm);
                        z_t.clear();
                    }
                }
            }
        }

        // Update mu_ and Sigma_
        std::tie(mu_, Sigma_, map_pipe_) = ekf_filter_->ekfUpdate();

        // Publish and broadcast
        this->sendOutput(ros::Time::now());
        this->bcMapOdomTF(ros::Time::now());
        this->updateMapMarkers(1.0);
        pipe_meas_cnt_ += 1;
    }
    else{
        ROS_WARN("No odometry info received, bc latest known map --> odom tf");
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
