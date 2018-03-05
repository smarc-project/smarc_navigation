#include "ekf_localization/ekf_localization.hpp"

// HELPER FUNCTIONS TODO: move to aux library
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

unsigned int factorial(unsigned int n)
{
    unsigned int ret = 1;
    if(n == 0) return 1;
    for(unsigned int i = 1; i <= n; ++i){
        ret *= i;
    }
    return ret;
}

bool sortLandmarksML(CorrespondenceClass *ml_1, CorrespondenceClass *ml_2){

    return (ml_1->psi_ > ml_2->psi_)? true: false;
}

// END HELPER FUNCTIONS


EKFLocalization::EKFLocalization(std::string node_name, ros::NodeHandle &nh): nh_(&nh), node_name_(node_name){

    std::string imu_topic;
    std::string dvl_topic;
    std::string odom_topic;
    std::string odom_in_topic;
    std::string gt_topic;
    std::string rpt_topic;
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
    nh_->param<std::string>((node_name_ + "/imu_topic"), imu_topic, "/imu");
    nh_->param<std::string>((node_name_ + "/dvl_topic"), dvl_topic, "/dvl");
    nh_->param<std::string>((node_name_ + "/odom_pub_topic"), odom_topic, "/odom_ekf");
    nh_->param<std::string>((node_name_ + "/odom_in_pub_topic"), odom_in_topic, "/odom_ekf");
    nh_->param<std::string>((node_name_ + "/gt_pose_topic"), gt_topic, "/gt_pose");
    nh_->param<std::string>((node_name_ + "/lm_detect_topic"), observs_topic, "/landmarks_detected");
    nh_->param<std::string>((node_name_ + "/rpt_topic"), rpt_topic, "/rpt_topic");
    nh_->param<std::string>((node_name_ + "/odom_frame"), odom_frame_, "/odom");
    nh_->param<std::string>((node_name_ + "/world_frame"), world_frame_, "/world");
    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/dvl_frame"), dvl_frame_, "/dvl_link");
    nh_->param<std::string>((node_name_ + "/map_srv"), map_srv_name_, "/gazebo/get_world_properties");
    nh_->param<std::string>((node_name_ + "/landmarks_srv"), lm_srv_name_, "/gazebo/get_model_state");

    // Synch IMU and DVL readings
    imu_subs_ = new message_filters::Subscriber<sensor_msgs::Imu>(*nh_, imu_topic, 25);
    dvl_subs_ = new message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>(*nh_, dvl_topic, 5);
    msg_synch_ptr_ = new message_filters::Synchronizer<MsgTimingPolicy> (MsgTimingPolicy(5), *imu_subs_, *dvl_subs_);
    msg_synch_ptr_->registerCallback(boost::bind(&EKFLocalization::synchSensorsCB, this, _1, _2));

    // Subscribe to sensor msgs
    fast_imu_sub_ = nh_->subscribe(imu_topic, 10, &EKFLocalization::fastIMUCB, this);
    fast_dvl_sub_ = nh_->subscribe(dvl_topic, 10, &EKFLocalization::fastDVLCB, this);
    observs_subs_ = nh_->subscribe(observs_topic, 10, &EKFLocalization::observationsCB, this);
    tf_gt_subs_ = nh_->subscribe(gt_topic, 10, &EKFLocalization::gtCB, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic, 10);
    odom_inertial_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_in_topic, 10);

    // Build world map from Gazebo
    gazebo_client_ = nh_->serviceClient<gazebo_msgs::GetWorldProperties>(map_srv_name_);
    landmarks_client_ = nh_->serviceClient<gazebo_msgs::GetModelState>(lm_srv_name_);

    // Plot map in RVIZ
    vis_pub_ = nh_->advertise<visualization_msgs::MarkerArray>( "/rviz/landmarks", 0 );

    // Initialize internal params
    init(Sigma_diagonal, R_diagonal, Q_diagonal, delta);

    // Main spin loop
    timer_ = nh_->createTimer(ros::Duration(1.0 / std::max(freq, 1.0)), &EKFLocalization::ekfLocalize, this);

}

void EKFLocalization::init(std::vector<double> sigma_diag, std::vector<double> r_diag, std::vector<double> q_diag, double delta){

    // EKF variables
    double size_state = r_diag.size();
    double size_meas = q_diag.size();
    mu_ = boost::numeric::ublas::zero_vector<double>(size_state);
//    mu_(1) = 2;
//    mu_(2) = 3;
    mu_pred_ = mu_;

    Sigma_ = boost::numeric::ublas::identity_matrix<double>(size_state);
    for(unsigned int i=0; i<size_state; i++){
        Sigma_(i,i) = sigma_diag.at(i);
    }
    R_ = boost::numeric::ublas::identity_matrix<double> (size_state);
    for(unsigned int i=0; i<size_state; i++){
        R_(i,i) = r_diag.at(i);
    }
    Q_ = boost::numeric::ublas::identity_matrix<double> (size_meas);
    for(unsigned int i=0; i<size_meas; i++){
        Q_(i,i) = q_diag.at(i);
    }

    // Outlier rejection
    delta_m_ = delta; // TODO: Add as rosparam
    boost::math::chi_squared chi2_dist(size_meas);
    lambda_M_ = boost::math::quantile(chi2_dist, delta_m_);

    // State machine
    init_filter_ = false;
    coord_ = false;

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

    // Get list of sim models from Gazebo
    while(!ros::service::waitForService(map_srv_name_, ros::Duration(10)) && ros::ok()){
        ROS_INFO_NAMED(node_name_,"Waiting for the gazebo world prop service to come up");
    }

    // Get states of the models from Gazebo (to build map)
    while(!ros::service::waitForService(lm_srv_name_, ros::Duration(10)) && ros::ok()){
        ROS_INFO_NAMED(node_name_,"Waiting for the gazebo model states service to come up");
    }

    // Build map for localization from Gazebo services and transform to odom frame coordinates
    gazebo_msgs::GetWorldProperties world_prop_srv;
    gazebo_msgs::GetModelState landmark_state_srv;
    tf::Vector3 lm_world;
    tf::Vector3 lm_odom;
    std::vector<boost::numeric::ublas::vector<double>> map_world;
    if(gazebo_client_.call(world_prop_srv)){
        int id = 0;
        boost::numeric::ublas::vector<double> aux_vec(4);
        for(auto landmark_name: world_prop_srv.response.model_names){
            if(landmark_name != "lolo_auv" && landmark_name != "ned" && landmark_name != "ocean" && landmark_name != "dummy_laser"){
                landmark_state_srv.request.model_name = landmark_name;
                if(landmarks_client_.call(landmark_state_srv)){
                    aux_vec(0) = id;

                    // Store map in world frame
                    aux_vec(1) = landmark_state_srv.response.pose.position.x;
                    aux_vec(2) = landmark_state_srv.response.pose.position.y;
                    aux_vec(3) = landmark_state_srv.response.pose.position.z;
                    map_world.push_back(aux_vec);

                    // Map in odom frame
                    lm_world = tf::Vector3(landmark_state_srv.response.pose.position.x,
                                           landmark_state_srv.response.pose.position.y,
                                           landmark_state_srv.response.pose.position.z);
                    lm_odom = transf_odom_world_ * lm_world;
                    aux_vec(1) = lm_odom.x();
                    aux_vec(2) = lm_odom.y();
                    aux_vec(3) = lm_odom.z();
                    map_odom_.push_back(aux_vec);
                    id++;
                }
            }
        }
    }
    createMapMarkers(map_world);

    // Create 1D KF to filter input sensors
//    dvl_x_kf = new OneDKF(0,0.1,0,0.001); // Adjust noise params for each filter
//    dvl_y_kf = new OneDKF(0,0.1,0,0.001);
//    dvl_z_kf = new OneDKF(0,0.1,0,0.001);

    ROS_INFO_NAMED(node_name_, "Initialized");
}

void EKFLocalization::observationsCB(const geometry_msgs::PoseArrayPtr &observ_msg){
    measurements_t_.push_back(observ_msg);
}

void EKFLocalization::fastIMUCB(const sensor_msgs::ImuPtr &imu_msg){
    imu_readings_.push_back(imu_msg);
    while(imu_readings_.size() > size_imu_q_){
        imu_readings_.pop_front();
    }
}

void EKFLocalization::fastDVLCB(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg){
//    dvl_x_kf->filter(dvl_msg->twist.twist.linear.x);
//    dvl_y_kf->filter(dvl_msg->twist.twist.linear.y);
//    dvl_z_kf->filter(dvl_msg->twist.twist.linear.z);

    boost::mutex::scoped_lock lock(msg_lock_);
    dvl_readings_.push_back(dvl_msg);
    while(dvl_readings_.size() > size_dvl_q_){
        dvl_readings_.pop_front();
    }
}

void EKFLocalization::synchSensorsCB(const sensor_msgs::ImuConstPtr &imu_msg,
                                    const geometry_msgs::TwistWithCovarianceStampedConstPtr &dvl_msg){
    coord_ = true;
}

void EKFLocalization::gtCB(const nav_msgs::OdometryPtr &pose_msg){
    gt_readings_.push_back(pose_msg);
    unsigned int size_gt_q = 10;
    while(gt_readings_.size() > size_gt_q){
        gt_readings_.pop_front();
    }
}

void EKFLocalization::createMapMarkers(std::vector<boost::numeric::ublas::vector<double>> map_world){

    unsigned int i = 0;
    for (auto landmark: map_world){
        visualization_msgs::Marker markers;
        markers.header.frame_id = "world";
        markers.header.stamp = ros::Time();
        markers.ns = "map_array";
        markers.id = i;
        markers.type = visualization_msgs::Marker::CUBE;
        markers.action = visualization_msgs::Marker::ADD;
        markers.pose.position.x = landmark(1);
        markers.pose.position.y = landmark(2);
        markers.pose.position.z = landmark(3);
        markers.pose.orientation.x = 0.0;
        markers.pose.orientation.y = 0.0;
        markers.pose.orientation.z = 0.0;
        markers.pose.orientation.w = 1.0;
        markers.scale.x = 1;
        markers.scale.y = 1;
        markers.scale.z = 1;
        markers.color.a = 1.0;
        markers.color.r = 0.0;
        markers.color.g = 1.0;
        markers.color.b = 0.0;

        markers_.markers.push_back(markers);
        i += 1;
    }
    std::cout << "number of landmars in map: " << i << std::endl;
}

bool EKFLocalization::sendOutput(ros::Time t){

    tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(mu_(3), mu_(4), mu_(5));
    q_auv_t.normalize();
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q_auv_t, odom_quat);

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

    nav_msgs::Odometry odom_inertial_msg;
    odom_inertial_msg.header.stamp = t;
    odom_inertial_msg.header.frame_id = odom_frame_;
    odom_inertial_msg.child_frame_id = base_frame_;
    odom_inertial_msg.pose.pose.position.x = mu_pred_(0);
    odom_inertial_msg.pose.pose.position.y = mu_pred_(1);
    odom_inertial_msg.pose.pose.position.z = mu_pred_(2);
    odom_inertial_msg.pose.pose.orientation = odom_quat;
    odom_inertial_pub_.publish(odom_inertial_msg);

    return true;
}

void EKFLocalization::interpolateDVL(ros::Time t_now, geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg_ptr){

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
                   std::pow(1 - (t_now.toSec() - dvl_readings_.at(n)->header.stamp.toSec())/
                            (dvl_readings_.at(n)->header.stamp.toSec() - dvl_readings_.at(n - n)->header.stamp.toSec()), n-l) *
                   std::pow((t_now.toSec() - dvl_readings_.at(n)->header.stamp.toSec())/
                            (dvl_readings_.at(n)->header.stamp.toSec() - dvl_readings_.at(n - n)->header.stamp.toSec()), l);
        u_interp.x += dvl_readings_.at(n - l)->twist.twist.linear.x * aux[l];
        u_interp.y += dvl_readings_.at(n - l)->twist.twist.linear.y * aux[l];
        u_interp.z += dvl_readings_.at(n - l)->twist.twist.linear.z * aux[l];
    }

    // New interpolated reading
    dvl_msg_ptr.reset(new geometry_msgs::TwistWithCovarianceStamped{});
    dvl_msg_ptr->header.stamp = t_now;
    dvl_msg_ptr->twist.twist.linear = u_interp;
}

void EKFLocalization::computeOdom(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg,
                                  const tf::Quaternion& q_auv, boost::numeric::ublas::vector<double> &u_t){

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
    double droll = angleLimit(roll_t - mu_(3));
    double dpitch = angleLimit(pitch_t - mu_(4));
    double dtheta = angleLimit(yaw_t - mu_(5));

    // Incremental part of the motion model
    u_t(0) = disp_odom.x();
    u_t(1) = disp_odom.y();
    u_t(2) = disp_odom.z();
    u_t(3) = droll;
    u_t(4) = dpitch;
    u_t(5) = dtheta;

    // Derivative of motion model in mu_ (t-1)
    using namespace std;
    G_t_ = boost::numeric::ublas::zero_matrix<double>(6);

    G_t_(0,0) = 1;
    G_t_(0,3) = disp_base.y()*(sin(roll_t)*sin(yaw_t) + cos(roll_t)*cos(yaw_t)*sin(pitch_t))
                + disp_base.z()*(cos(roll_t)*sin(yaw_t) - cos(yaw_t)*sin(pitch_t)*sin(roll_t));
    G_t_(0,4) = cos(yaw_t)*(disp_base.z()*cos(pitch_t)*cos(roll_t) - disp_base.x()*sin(pitch_t)
                + disp_base.y()*cos(pitch_t)*sin(roll_t));
    G_t_(0,5) = disp_base.z()*(cos(yaw_t)*sin(roll_t) - cos(roll_t)*sin(pitch_t)*sin(yaw_t))
                - disp_base.y()*(cos(roll_t)*cos(yaw_t) + sin(pitch_t)*sin(roll_t)*sin(yaw_t))
                - disp_base.x()*cos(pitch_t)*sin(yaw_t);

    G_t_(1,1) = 1;
    G_t_(1,3) = - disp_base.y()*(cos(yaw_t)*sin(roll_t) - cos(roll_t)*sin(pitch_t)*sin(yaw_t))
                - disp_base.z()*(cos(roll_t)*cos(yaw_t) + sin(pitch_t)*sin(roll_t)*sin(yaw_t));
    G_t_(1,4) = sin(yaw_t)*(disp_base.z()*cos(pitch_t)*cos(roll_t) - disp_base.x()*sin(pitch_t)
                + disp_base.y()*cos(pitch_t)*sin(roll_t));
    G_t_(1,5) = disp_base.z()*(sin(roll_t)*sin(yaw_t) + cos(roll_t)*cos(yaw_t)*sin(pitch_t))
               - disp_base.y()*(cos(roll_t)*sin(yaw_t) - cos(yaw_t)*sin(pitch_t)*sin(roll_t))
               + disp_base.x()*cos(pitch_t)*cos(yaw_t);

    G_t_(2,2) = 1;
    G_t_(2,3) = cos(pitch_t)*(disp_base.y()*cos(roll_t) - disp_base.z()*sin(roll_t));
    G_t_(2,4) = - disp_base.x()*cos(pitch_t) - disp_base.z()*cos(roll_t)*sin(pitch_t)
                - disp_base.y()*sin(pitch_t)*sin(roll_t);
    G_t_(2,5) = 0;

    t_prev_ = t_now;
}

void EKFLocalization::predictMotion(boost::numeric::ublas::vector<double> &u_t){

    // Compute predicted mu
    mu_hat_ = mu_ + u_t;
    mu_hat_(3) = angleLimit(mu_hat_(3));
    mu_hat_(4) = angleLimit(mu_hat_(4));
    mu_hat_(5) = angleLimit(mu_hat_(5));
    mu_pred_ += u_t;

    // Predicted covariance matrix
    boost::numeric::ublas::matrix<double> aux = boost::numeric::ublas::prod(G_t_, Sigma_);
    Sigma_hat_ = boost::numeric::ublas::prod(aux, boost::numeric::ublas::trans(G_t_));
    Sigma_hat_ += R_;
}

void EKFLocalization::predictMeasurement(const boost::numeric::ublas::vector<double> &landmark_j,
                                      boost::numeric::ublas::vector<double> &z_i,
                                      std::vector<CorrespondenceClass *> &ml_i_list){

    using namespace boost::numeric::ublas;

    // Compute transform odom --> base from current state estimate
    tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(mu_hat_(3), mu_hat_(4), mu_hat_(5));
    q_auv_t.normalize();
    tf::Transform transf_odom_base = tf::Transform(q_auv_t, tf::Vector3(mu_hat_(0), mu_hat_(1), mu_hat_(2)));

    // Measurement model: z_hat_i
    tf::Vector3 landmark_j_odom = tf::Vector3(landmark_j(1),
                                              landmark_j(2),
                                              landmark_j(3));
    tf::Vector3 z_hat_sss;
    z_hat_sss = transf_odom_base.inverse() * landmark_j_odom;

    vector<double> z_i_hat_base = vector<double>(3);
    z_i_hat_base(0) = z_hat_sss.getX();
    z_i_hat_base(1) = z_hat_sss.getY();
    z_i_hat_base(2) = z_hat_sss.getZ();

    // Compute ML of observation z_i with M_j
    CorrespondenceClass *corresp_j_ptr;
    corresp_j_ptr = new CorrespondenceClass(landmark_j(0));
    corresp_j_ptr->computeH(mu_hat_, landmark_j_odom);
    corresp_j_ptr->computeS(Sigma_hat_, Q_);
    corresp_j_ptr->computeNu(z_i_hat_base, z_i);
    corresp_j_ptr->computeLikelihood();

    // Outlier rejection
    if(corresp_j_ptr->d_m_ < lambda_M_){
        ml_i_list.push_back(corresp_j_ptr);
    }
    else{
        ROS_DEBUG_NAMED(node_name_, "Outlier rejected");
    }
}

void EKFLocalization::dataAssociation(){
    boost::numeric::ublas::vector<double> z_i(3);
    std::vector<boost::numeric::ublas::vector<double>> z_t;

    double epsilon = 10;
    // If observations available
    if(!measurements_t_.empty()){
//        ROS_INFO("-----Measurements received-----");
        for(auto observ: measurements_t_){  //TODO: it should be only one z_t per measurement update
            // Compensate for the volume of the stones*****
            for(auto lm_pose: observ->poses){
                z_i(0) = lm_pose.position.x;
                z_i(1) = lm_pose.position.y - 1/std::sqrt(2);
                z_i(2) = lm_pose.position.z - 1/std::sqrt(2);
                z_t.push_back(z_i);
            }
        }
        measurements_t_.pop_front();
        if(!measurements_t_.empty()){
            ROS_WARN("Cache with measurements is not empty");
        }
        // Main ML loop
        std::vector<CorrespondenceClass*> ml_i_list;
        int cnt = 0;
        // For each observation z_i at time t
        for(auto z_i: z_t){
            // For each possible landmark j in M
            for(auto landmark_j: map_odom_){
                // Narrow down the landmarks to be checked
                if(epsilon > std::abs((landmark_j(1) - mu_hat_(0)) + (mu_hat_(1) - landmark_j(2)) / std::tan(angleLimit(M_PI/2.0 + mu_hat_(5))))){
                    predictMeasurement(landmark_j, z_i, ml_i_list);
                    cnt += 1;
                }
            }
            // Select the association with the maximum likelihood
            if(!ml_i_list.empty()){
                if(ml_i_list.size() > 1){
                    std::sort(ml_i_list.begin(), ml_i_list.end(), sortLandmarksML);
                }
                // Sequential update
                sequentialUpdate(ml_i_list.front());
                ml_i_list.clear();
            }
        }
    }
}

void EKFLocalization::sequentialUpdate(CorrespondenceClass* c_i_j){

    using namespace boost::numeric::ublas;
    matrix<double> K_t_i;
    matrix<double> H_trans;
    identity_matrix<double> I(Sigma_hat_.size1(), Sigma_hat_.size2());
    matrix<double> aux_mat;

    // Compute Kalman gain
    H_trans = trans(c_i_j->H_);
    K_t_i = prod(Sigma_hat_, H_trans);
    K_t_i = prod(K_t_i, c_i_j->S_inverted_);
    // Update mu_hat and sigma_hat
    mu_hat_ += prod(K_t_i, c_i_j->nu_);
    mu_hat_(3) = angleLimit(mu_hat_(3));
    mu_hat_(4) = angleLimit(mu_hat_(4));
    mu_hat_(5) = angleLimit(mu_hat_(5));
    aux_mat = (I  - prod(K_t_i, c_i_j->H_));
    Sigma_hat_ = prod(aux_mat, Sigma_hat_);
}

void EKFLocalization::ekfLocalize(const ros::TimerEvent& e){

    sensor_msgs::ImuPtr imu_msg;
    geometry_msgs::TwistWithCovarianceStampedPtr dvl_msg;
    nav_msgs::OdometryPtr gt_msg;

    tf::Quaternion q_auv;
    boost::numeric::ublas::vector<double> u_t = boost::numeric::ublas::vector<double>(6);

    if(dvl_readings_.size() >= size_dvl_q_ && imu_readings_.size() >= size_imu_q_ && !gt_readings_.empty()){
        // Init filter with initial, true pose (from GPS?)
        if(!init_filter_){
            ROS_INFO_NAMED(node_name_, "Starting localization node");

            // Compute initial pose
            gt_msg = gt_readings_.back();
            t_prev_ = gt_msg->header.stamp.toSec();

            // Transform IMU output world --> odom
            tf::Quaternion q_transf;
            tf::quaternionMsgToTF(gt_msg->pose.pose.orientation, q_transf);
            q_auv = transf_odom_world_.getRotation() * q_transf;
            q_auv.normalize();

            // Publish and broadcast
            this->sendOutput(gt_msg->header.stamp);

            init_filter_ = true;
        }
        // MAIN LOOP
        else{
            // Fetch latest sensor readings
            imu_msg = imu_readings_.back();

            if(coord_ == false){
                // IMU available but not DVL
                this->interpolateDVL(imu_msg->header.stamp, dvl_msg);
            }
            else{
                // Sensor input available on both channels
                coord_ = false;
                dvl_msg = dvl_readings_.back();
            }

            // Transform IMU output world --> odom
            tf::Quaternion q_transf;
            tf::quaternionMsgToTF(imu_msg->orientation, q_transf);
            q_auv = transf_odom_world_.getRotation() * q_transf;
            q_auv.normalize();

            // Compute displacement based on DVL and IMU orientation
            computeOdom(dvl_msg, q_auv, u_t);

            // Prediction step
            predictMotion(u_t);

            // Data association and sequential update
            dataAssociation();

            // Update step
            mu_ = mu_hat_;
            mu_(3) = angleLimit(mu_(3));
            mu_(4) = angleLimit(mu_(4));
            mu_(5) = angleLimit(mu_(5));
            Sigma_ = Sigma_hat_;

            // Publish and broadcast
            this->sendOutput(dvl_msg->header.stamp);
            vis_pub_.publish(markers_);
        }
    }
    else{
        gt_msg = gt_readings_.back();
        this->sendOutput(gt_msg->header.stamp);
        ROS_INFO("No sensory update, broadcasting latest known pose");
    }

}

EKFLocalization::~EKFLocalization(){
    // TODO_NACHO: do some cleaning here
}
