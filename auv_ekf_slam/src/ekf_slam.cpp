#include "ekf_slam/ekf_slam.hpp"

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
    mu_.setZero(size_state);
    mu_pred_ = mu_;
    lm_num_ = 0;
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
    delta_m_ = delta;
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

    // Create 1D KF to filter input sensors
//    dvl_x_kf = new OneDKF(0,0.1,0,0.001); // Adjust noise params for each filter
//    dvl_y_kf = new OneDKF(0,0.1,0,0.001);
//    dvl_z_kf = new OneDKF(0,0.1,0,0.001);

    ROS_INFO_NAMED(node_name_, "Initialized");
}

void EKFLocalization::observationsCB(const geometry_msgs::PoseArray &observ_msg){
    measurements_t_.push_back(observ_msg);
}

void EKFLocalization::fastIMUCB(const sensor_msgs::Imu &imu_msg){
    imu_readings_.push_back(imu_msg);
    while(imu_readings_.size() > size_imu_q_){
        imu_readings_.pop_front();
    }
}

void EKFLocalization::fastDVLCB(const geometry_msgs::TwistWithCovarianceStamped &dvl_msg){
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

void EKFLocalization::gtCB(const nav_msgs::Odometry &pose_msg){
    gt_readings_.push_back(pose_msg);
    unsigned int size_gt_q = 10;
    while(gt_readings_.size() > size_gt_q){
        gt_readings_.pop_front();
    }
}

void EKFLocalization::updateMapMarkers(double color){

    visualization_msgs::MarkerArray marker_array;
    Eigen::Vector3d landmark;
    for(unsigned int j=0; j<(mu_.rows()-6)/3; j++){
        landmark = mu_.segment(3 * j + 6, 3);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
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

void EKFLocalization::computeOdom(const geometry_msgs::TwistWithCovarianceStampedPtr &dvl_msg,
                                  const tf::Quaternion& q_auv, Eigen::VectorXd &u_t,
                                  Eigen::MatrixXd &g_t){

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
    g_t.setZero(6, 6);

    g_t(0,3) = disp_base.y()*(sin(roll_t)*sin(yaw_t) + cos(roll_t)*cos(yaw_t)*sin(pitch_t))
                + disp_base.z()*(cos(roll_t)*sin(yaw_t) - cos(yaw_t)*sin(pitch_t)*sin(roll_t));
    g_t(0,4) = cos(yaw_t)*(disp_base.z()*cos(pitch_t)*cos(roll_t) - disp_base.x()*sin(pitch_t)
                + disp_base.y()*cos(pitch_t)*sin(roll_t));
    g_t(0,5) = disp_base.z()*(cos(yaw_t)*sin(roll_t) - cos(roll_t)*sin(pitch_t)*sin(yaw_t))
                - disp_base.y()*(cos(roll_t)*cos(yaw_t) + sin(pitch_t)*sin(roll_t)*sin(yaw_t))
                - disp_base.x()*cos(pitch_t)*sin(yaw_t);

    g_t(1,3) = - disp_base.y()*(cos(yaw_t)*sin(roll_t) - cos(roll_t)*sin(pitch_t)*sin(yaw_t))
                - disp_base.z()*(cos(roll_t)*cos(yaw_t) + sin(pitch_t)*sin(roll_t)*sin(yaw_t));
    g_t(1,4) = sin(yaw_t)*(disp_base.z()*cos(pitch_t)*cos(roll_t) - disp_base.x()*sin(pitch_t)
                + disp_base.y()*cos(pitch_t)*sin(roll_t));
    g_t(1,5) = disp_base.z()*(sin(roll_t)*sin(yaw_t) + cos(roll_t)*cos(yaw_t)*sin(pitch_t))
               - disp_base.y()*(cos(roll_t)*sin(yaw_t) - cos(yaw_t)*sin(pitch_t)*sin(roll_t))
               + disp_base.x()*cos(pitch_t)*cos(yaw_t);

    g_t(2,3) = cos(pitch_t)*(disp_base.y()*cos(roll_t) - disp_base.z()*sin(roll_t));
    g_t(2,4) = - disp_base.x()*cos(pitch_t) - disp_base.z()*cos(roll_t)*sin(pitch_t)
                - disp_base.y()*sin(pitch_t)*sin(roll_t);
    g_t(2,5) = 0;

    t_prev_ = t_now;
}

void EKFLocalization::predictMotion(Eigen::VectorXd &u_t,
                                    Eigen::MatrixXd &g_t){

    // Construct Fx (6,3N) for dimension mapping
    Eigen::SparseMatrix<double> F_x(6, 6 + 3*lm_num_);
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(6);
    unsigned int j;
    int input = 1;
    for(unsigned int i=0; i<6; i++){
        j = i;
        tripletList.push_back(Eigen::Triplet<double>(i,j,input));
    }
    F_x.setFromTriplets(tripletList.begin(), tripletList.end());
    Eigen::SparseMatrix<double> F_x_transp = F_x.transpose();

    // Compute predicted mu
    mu_hat_ = mu_ + F_x_transp * u_t;
    mu_hat_(3) = angleLimit(mu_hat_(3));
    mu_hat_(4) = angleLimit(mu_hat_(4));
    mu_hat_(5) = angleLimit(mu_hat_(5));
    mu_pred_ += u_t;

    // Compute Jacobian G_t
    Eigen::MatrixXd G_t = Eigen::MatrixXd::Identity(6 + 3*lm_num_, 6 + 3*lm_num_);
    G_t(3,3) = 0;   // G_t is zero here because the motion model uses abs values for RPY
    G_t(4,4) = 0;
    G_t(5,5) = 0;
    G_t += F_x_transp * g_t * F_x;

    // Predicted covariance matrix
    Sigma_hat_ = G_t * Sigma_ * G_t.transpose();
    Sigma_hat_ += F_x_transp * R_ * F_x;

}

void EKFLocalization::predictMeasurement(const Eigen::Vector3d &landmark_j,
                                         const Eigen::Vector3d &z_i,
                                         unsigned int i,
                                         unsigned int j,
                                         const tf::Transform &transf_base_odom,
                                         const Eigen::MatrixXd &temp_sigma,
                                         h_comp h_comps,
                                         std::vector<CorrespondenceClass> &corresp_i_list){

    using namespace boost::numeric::ublas;
    //    auto (re1, re2, re3) = myfunc(2);

    // Measurement model: z_hat_i
    tf::Vector3 landmark_j_odom(landmark_j(0),
                                landmark_j(1),
                                landmark_j(2));

    tf::Vector3 z_hat_base = transf_base_odom * landmark_j_odom;
    Eigen::Vector3d z_k_hat_base( z_hat_base.getX(),
                                  z_hat_base.getY(),
                                  z_hat_base.getZ());

    // Compute ML of observation z_i with M_j
    CorrespondenceClass corresp_i_j(i, j);
    corresp_i_j.computeH(h_comps, landmark_j_odom);
    corresp_i_j.computeNu(z_k_hat_base, z_i);
    corresp_i_j.computeMHLDistance(temp_sigma, Q_);

    // Outlier rejection
    if(corresp_i_j.d_m_ < lambda_M_){
        corresp_i_list.push_back(std::move(corresp_i_j));
    }
    else{
        ROS_DEBUG_NAMED(node_name_, "Outlier rejected");
    }
}

void EKFLocalization::dataAssociation(){
    std::vector<Eigen::Vector3d> z_t;

    double epsilon = 10;
    double alpha = 0.085;   // TODO: find suitable value!!

    // If observations available
    if(!measurements_t_.empty()){
        // Fetch latest measurement
        auto observ = measurements_t_.back();   // TODO: make sure the system uses the latest measurement
        measurements_t_.pop_back();

        for(auto lm_pose: observ.poses){
            z_t.push_back(Eigen::Vector3d(lm_pose.position.x,
                                          lm_pose.position.y,
                                          lm_pose.position.z));
        }
        if(!measurements_t_.empty()){
            ROS_WARN("Cache with measurements is not empty");
        }

        // Main loop
        std::vector<CorrespondenceClass> corresp_i_list;
        tf::Vector3 new_lm_aux;
        tf::Transform transf_base_odom;
        tf::Transform transf_odom_base;
        Eigen::MatrixXd temp_sigma(9,9);

        lm_num_ = (mu_.rows() - 6) / 3;
        // For each observation z_i at time t
        for(unsigned int i = 0; i<z_t.size(); i++){
            // Compute transform odom --> base from current state state estimate at time t
            transf_odom_base = tf::Transform(tf::createQuaternionFromRPY(mu_hat_(3), mu_hat_(4), mu_hat_(5)).normalize(),
                                             tf::Vector3(mu_hat_(0), mu_hat_(1), mu_hat_(2)));
            transf_base_odom = transf_odom_base.inverse();

            // Back-project new possible landmark (in odom frame)
            new_lm_aux = transf_odom_base * tf::Vector3(z_t.at(i)(0), z_t.at(i)(1),z_t.at(i)(2));

            // Add new possible landmark to mu_hat_
            Eigen::VectorXd aux_mu = mu_hat_;
            mu_hat_.resize(mu_hat_.size()+3, true);
            mu_hat_ << aux_mu, Eigen::Vector3d(new_lm_aux.getX(),
                                               new_lm_aux.getY(),
                                               new_lm_aux.getZ());

            // Increase Sigma_hat_
            Sigma_hat_.conservativeResize(Sigma_hat_.rows()+3, Sigma_hat_.cols()+3);
            Sigma_hat_.bottomRows(3).setZero();
            Sigma_hat_.rightCols(3).setZero();
            Sigma_hat_(Sigma_hat_.rows()-3, Sigma_hat_.cols()-3) = 10;  // TODO: initialize with uncertainty on the measurement in x,y,z
            Sigma_hat_(Sigma_hat_.rows()-2, Sigma_hat_.cols()-2) = 10;
            Sigma_hat_(Sigma_hat_.rows()-1, Sigma_hat_.cols()-1) = 100;

            // Store current mu_hat_ estimate in struct for faster computation of H in DA
            h_comp h_comps;
            {
                using namespace std;
                h_comps.mu_0 = mu_hat_(0);
                h_comps.mu_1 = mu_hat_(1);
                h_comps.mu_2 = mu_hat_(2);
                h_comps.c_3 = cos(mu_hat_(3));
                h_comps.c_4 = cos(mu_hat_(4));
                h_comps.c_5 = cos(mu_hat_(5));
                h_comps.s_3 = sin(mu_hat_(3));
                h_comps.s_4 = sin(mu_hat_(4));
                h_comps.s_5 = sin(mu_hat_(5));
            }

            // Store block of sigma common to all landmarks analysis
            temp_sigma.block(0,0,6,6) = Sigma_hat_.block(0,0,6,6);
            // For each possible landmark j in M
            Eigen::Vector3d landmark_j;
            for(unsigned int j=0; j<(mu_hat_.rows()-6)/3; j++){
                landmark_j = mu_hat_.segment(3 * j + 6, 3);
                temp_sigma.bottomRows(3) = Sigma_hat_.block(j * 3 + 6, 0, 3, temp_sigma.cols());
                temp_sigma.rightCols(3) = Sigma_hat_.block(0, j * 3 + 6, temp_sigma.rows(), 3);
                predictMeasurement(landmark_j, z_t.at(i), i, j + 1, transf_base_odom, temp_sigma, h_comps, corresp_i_list);
            }

            // Select the association with the minimum Mahalanobis distance
            if(!corresp_i_list.empty()){

                // Set init Mahalanobis distance for new possible landmark
                corresp_i_list.back().d_m_ = alpha;

                // Select correspondance with minimum Mh distance
                std::sort(corresp_i_list.begin(), corresp_i_list.end(), [](const CorrespondenceClass& corresp_1, const CorrespondenceClass& corresp_2){
                    return corresp_1.d_m_ > corresp_2.d_m_;
                });

                // Update landmarks in the map
                if(lm_num_ >= corresp_i_list.back().i_j_.second){
                    // No new landmark added --> remove candidate from mu_hat_ and sigma_hat_
                    mu_hat_.conservativeResize(mu_hat_.rows()-3);
                    Sigma_hat_.conservativeResize(Sigma_hat_.rows()-3, Sigma_hat_.cols()-3);
                    temp_sigma.bottomRows(3) = Sigma_hat_.block((corresp_i_list.back().i_j_.second - 1) * 3 + 6, 0, 3, temp_sigma.cols());
                    temp_sigma.rightCols(3) = Sigma_hat_.block(0, (corresp_i_list.back().i_j_.second - 1) * 3 + 6, temp_sigma.rows(), 3);
                    sequentialUpdate(corresp_i_list.back(), temp_sigma);
                }
                else{
                    // New landmark
                    lm_num_ = corresp_i_list.back().i_j_.second;
                }
                // Sequential update
                corresp_i_list.clear();
            }
        }
        // Make sure mu and sigma have the same size at the end!
        while(mu_hat_.size() < Sigma_hat_.rows()){
            ROS_WARN("Sizes of mu and sigma differ!!");
            Sigma_hat_.conservativeResize(Sigma_hat_.rows()-3, Sigma_hat_.cols()-3);
        }
    }
}

void EKFLocalization::sequentialUpdate(CorrespondenceClass const& c_i_j, Eigen::MatrixXd temp_sigma){

    // Compute Kalman gain
    temp_sigma.bottomRows(3) = Sigma_hat_.block((c_i_j.i_j_.second - 1) * 3 + 6, 0, 3, temp_sigma.cols());
    temp_sigma.rightCols(3) = Sigma_hat_.block( 0, (c_i_j.i_j_.second - 1) * 3 + 6, temp_sigma.rows(), 3);

    Eigen::MatrixXd K_t_i = temp_sigma * c_i_j.H_t_.transpose() * c_i_j.S_inverted_;

    // Update mu_hat and sigma_hat
    Eigen::VectorXd aux_vec = K_t_i * c_i_j.nu_;
    mu_hat_.head(6) += aux_vec.head(6);
    mu_hat_(3) = angleLimit(mu_hat_(3));
    mu_hat_(4) = angleLimit(mu_hat_(4));
    mu_hat_(5) = angleLimit(mu_hat_(5));
    mu_hat_.segment((c_i_j.i_j_.second - 1) * 3 + 6, 3) += aux_vec.segment(6, 3);

    Eigen::MatrixXd aux_mat = (Eigen::MatrixXd::Identity(temp_sigma.rows(), temp_sigma.cols()) - K_t_i * c_i_j.H_t_) * temp_sigma;
    Sigma_hat_.block(0,0,6,6) = aux_mat.block(0,0,6,6);
    Sigma_hat_.block(0, (c_i_j.i_j_.second - 1) * 3 + 6, temp_sigma.rows(), 3) = aux_mat.block(0, aux_mat.cols()-3, aux_mat.rows(), 3);
    Sigma_hat_.block((c_i_j.i_j_.second - 1) * 3 + 6, 0, 3, temp_sigma.cols()) = aux_mat.block(aux_mat.rows()-3, 0, 3, aux_mat.cols());
}

void EKFLocalization::ekfLocalize(const ros::TimerEvent& e){

    sensor_msgs::ImuPtr imu_msg;
    geometry_msgs::TwistWithCovarianceStampedPtr dvl_msg;
    nav_msgs::OdometryPtr gt_msg;

    // TODO: predefine matrices so that they can be allocated in the stack!
    tf::Quaternion q_auv;
    Eigen::VectorXd u_t(6);
    Eigen::MatrixXd g_t(6,6);

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

            // Publish and broadcast
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
            computeOdom(dvl_msg, q_auv, u_t, g_t);

            // Prediction step
            predictMotion(u_t, g_t);

            // Data association and sequential update
            //dataAssociation();

            // Update step
            if (mu_.rows()!= mu_hat_.rows()){
                int n_t = mu_hat_.rows() - mu_.rows();
                mu_.conservativeResize(mu_.size() + n_t, true);
                Sigma_.conservativeResize(Sigma_.rows() + n_t, Sigma_.cols() + n_t);
                std::cout << "Mu updated: " << mu_.size() << std::endl;
                std::cout << "Sigma updated: " << Sigma_.cols() << std::endl;
                std::cout << "Number of landmarks: " << (Sigma_.rows() - 6) / 3 << std::endl;
                // TODO: check that Sigma_ is still semi-definite positive
            }
            mu_ = mu_hat_;
            Sigma_ = Sigma_hat_;

            // Publish and broadcast
            this->sendOutput(dvl_msg->header.stamp);
            this->updateMapMarkers(1.0);
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

EKFLocalization::~EKFLocalization(){
    // Empty queues
    imu_readings_.clear();
    measurements_t_.clear();
    dvl_readings_.clear();
    gt_readings_.clear();

    // Delete instance pointers
    delete(nh_);
    delete(msg_synch_ptr_);
    delete(imu_subs_);
    delete(dvl_subs_);
}
