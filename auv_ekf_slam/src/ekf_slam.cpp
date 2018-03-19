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
    observs_subs_ = nh_->subscribe(observs_topic, 10, &EKFLocalization::observationsCB, this);
    odom_subs_ = nh_->subscribe(odom_topic, 10, &EKFLocalization::odomCB, this);
    map_pub_ = nh_->advertise<nav_msgs::Odometry>(map_topic, 10);

    // Plot map in RVIZ
    vis_pub_ = nh_->advertise<visualization_msgs::MarkerArray>( "/lolo_auv/rviz/landmarks", 0 );

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
    mu_hat_ = mu_;
    mu_auv_odom_.setZero(3);
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
    size_odom_q_ = 10;

    // Get fixed transform world --> odom frame
//    tf::TransformListener tf_listener;
//    try {
//        tf_listener.waitForTransform(world_frame_, odom_frame_, ros::Time(0), ros::Duration(10.0) );
//        tf_listener.lookupTransform(world_frame_, odom_frame_, ros::Time(0), transf_world_odom_);
//        ROS_INFO("Locked transform world --> odom");
//        // Compute inverse for later use
//        transf_odom_world_ = transf_world_odom_.inverse();
//    }
//    catch(tf::TransformException &exception) {
//        ROS_ERROR("%s", exception.what());
//        ros::Duration(1.0).sleep();
//    }

    ROS_INFO_NAMED(node_name_, "Initialized");
}

void EKFLocalization::odomCB(const nav_msgs::Odometry &odom_msg){
    odom_queue_t_.push_back(odom_msg);
    while(odom_queue_t_.size() > size_odom_q_){
        odom_queue_t_.pop_front();
    }
}

void EKFLocalization::observationsCB(const geometry_msgs::PoseArray &observ_msg){
    measurements_t_.push_back(observ_msg);
}


void EKFLocalization::updateMapMarkers(double color){

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

bool EKFLocalization::sendOutput(ros::Time t){

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

bool EKFLocalization::bcMapOdomTF(ros::Time t){
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


void EKFLocalization::predictMotion(nav_msgs::Odometry odom_reading){

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

    // Extract latest orientation from odom at time t
    tf::Quaternion q_odom;
    tf::quaternionMsgToTF(odom_reading.pose.pose.orientation, q_odom);
    q_odom.normalize();

    // Compute predicted mu_hat
    Eigen::Vector3d u_t(odom_reading.pose.pose.position.x,
                        odom_reading.pose.pose.position.y,
                        odom_reading.pose.pose.position.z);

    u_t -= mu_auv_odom_;    // Increment in x,y,z
    mu_hat_.segment(0, 3) = mu_.segment(0,3);
    mu_hat_.segment(0, 3) += u_t;
    mu_auv_odom_ += u_t;

    // Global orientation roll, pitch and yaw
    tf::Quaternion q(
        odom_reading.pose.pose.orientation.x,
        odom_reading.pose.pose.orientation.y,
        odom_reading.pose.pose.orientation.z,
        odom_reading.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(mu_hat_(3), mu_hat_(4), mu_hat_(5));
    mu_hat_(3) = angleLimit(mu_hat_(3));
    mu_hat_(4) = angleLimit(mu_hat_(4));
    mu_hat_(5) = angleLimit(mu_hat_(5));

    // Derivative of motion model in mu_ (t-1)
    Eigen::MatrixXd g_t(6,6);
    g_t.setZero(6, 6);
    using namespace std;

    g_t(0,3) = u_t(1)*(sin(mu_hat_(3))*sin(mu_hat_(5)) + cos(mu_hat_(3))*cos(mu_hat_(5))*sin(mu_hat_(4)))
                + u_t(2)*(cos(mu_hat_(3))*sin(mu_hat_(5)) - cos(mu_hat_(5))*sin(mu_hat_(4))*sin(mu_hat_(3)));
    g_t(0,4) = cos(mu_hat_(5))*(u_t(2)*cos(mu_hat_(4))*cos(mu_hat_(3)) - u_t(0)*sin(mu_hat_(4))
                + u_t(1)*cos(mu_hat_(4))*sin(mu_hat_(3)));
    g_t(0,5) = u_t(2)*(cos(mu_hat_(5))*sin(mu_hat_(3)) - cos(mu_hat_(3))*sin(mu_hat_(4))*sin(mu_hat_(5)))
                - u_t(1)*(cos(mu_hat_(3))*cos(mu_hat_(5)) + sin(mu_hat_(4))*sin(mu_hat_(3))*sin(mu_hat_(5)))
                - u_t(0)*cos(mu_hat_(4))*sin(mu_hat_(5));

    g_t(1,3) = - u_t(1)*(cos(mu_hat_(5))*sin(mu_hat_(3)) - cos(mu_hat_(3))*sin(mu_hat_(4))*sin(mu_hat_(5)))
                - u_t(2)*(cos(mu_hat_(3))*cos(mu_hat_(5)) + sin(mu_hat_(4))*sin(mu_hat_(3))*sin(mu_hat_(5)));
    g_t(1,4) = sin(mu_hat_(5))*(u_t(2)*cos(mu_hat_(4))*cos(mu_hat_(3)) - u_t(0)*sin(mu_hat_(4))
                + u_t(1)*cos(mu_hat_(4))*sin(mu_hat_(3)));
    g_t(1,5) = u_t(2)*(sin(mu_hat_(3))*sin(mu_hat_(5)) + cos(mu_hat_(3))*cos(mu_hat_(5))*sin(mu_hat_(4)))
               - u_t(1)*(cos(mu_hat_(3))*sin(mu_hat_(5)) - cos(mu_hat_(5))*sin(mu_hat_(4))*sin(mu_hat_(3)))
               + u_t(0)*cos(mu_hat_(4))*cos(mu_hat_(5));

    g_t(2,3) = cos(mu_hat_(4))*(u_t(1)*cos(mu_hat_(3)) - u_t(2)*sin(mu_hat_(3)));
    g_t(2,4) = - u_t(0)*cos(mu_hat_(4)) - u_t(2)*cos(mu_hat_(3))*sin(mu_hat_(4))
                - u_t(1)*sin(mu_hat_(4))*sin(mu_hat_(3));
    g_t(2,5) = 0;

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
        ROS_INFO_NAMED(node_name_, "Outlier rejected");
    }
}

void EKFLocalization::dataAssociation(std::vector<Eigen::Vector3d> z_t){

    double epsilon = 9;
    double alpha = 0.09;   // TODO: find suitable value!!

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
        Sigma_hat_(Sigma_hat_.rows()-3, Sigma_hat_.cols()-3) = 1000;  // TODO: initialize with uncertainty on the measurement in x,y,z
        Sigma_hat_(Sigma_hat_.rows()-2, Sigma_hat_.cols()-2) = 200;
        Sigma_hat_(Sigma_hat_.rows()-1, Sigma_hat_.cols()-1) = 200;

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
            }
            else{
                // New landmark
                lm_num_ = corresp_i_list.back().i_j_.second;
            }
            // Sequential update
            sequentialUpdate(corresp_i_list.back(), temp_sigma);
            corresp_i_list.clear();
        }
    }
    // Make sure mu and sigma have the same size at the end!
    while(mu_hat_.size() < Sigma_hat_.rows()){
        ROS_WARN("Sizes of mu and sigma differ!!");
        Sigma_hat_.conservativeResize(Sigma_hat_.rows()-3, Sigma_hat_.cols()-3);
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

    // TODO: predefine matrices so that they can be allocated in the stack!
    nav_msgs::Odometry odom_reading;
    std::vector<Eigen::Vector3d> z_t;

    if(!odom_queue_t_.empty()){
        // Fetch latest measurement
        odom_reading = odom_queue_t_.back();
        odom_queue_t_.pop_back();

        // Prediction step
        predictMotion(odom_reading);

        // If observations available: data association and sequential update
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
            dataAssociation(z_t);
            z_t.clear();
        }

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

EKFLocalization::~EKFLocalization(){
    // Empty queues
    measurements_t_.clear();
    odom_queue_t_.clear();

    // Delete instance pointers
    delete(nh_);
}
