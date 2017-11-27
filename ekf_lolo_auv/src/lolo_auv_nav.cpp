#include "lolo_auv_nav/lolo_auv_nav.hpp"

LoLoEKF::LoLoEKF(std::string node_name, ros::NodeHandle &nh):GeneralEKF(node_name, nh){

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
    imu_subs_ = nh_->subscribe(imu_topic, 10, &LoLoEKF::imuCB, this);
    dvl_subs_ = nh_->subscribe(dvl_topic, 10, &LoLoEKF::dvlCB, this);
    tf_gt_subs_ = nh_->subscribe(gt_topic, 10, &LoLoEKF::gtCB, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic, 10);

    // TODO: necessary to clear up queue at init?
    while(!this->imu_readings_.empty()){
        imu_readings_.pop();
    }
}

void LoLoEKF::init(const double &initial_R, const double &initial_Q, const double &delta, const unsigned int &size_n, const unsigned int &size_k){
    // Noise models params
    R_ = boost::numeric::ublas::identity_matrix<double> (size_n, size_n) * initial_R;
    Q_ = boost::numeric::ublas::identity_matrix<double> (size_k, size_k) * initial_Q;
    // Outlier rejection threshold
    delta_m_ = delta;
    boost::math::chi_squared chi2_dist(2);
    lambda_M_ = boost::math::quantile(chi2_dist, delta_m_);
    // Initial control variables
    u_t_ =boost::numeric::ublas::vector<double>(size_n);

    sigma_imu_ = boost::numeric::ublas::identity_matrix<double>(3) * 0.0000001;
    mu_imu_ = boost::numeric::ublas::vector<double>(3);

}

double LoLoEKF::angleLimit (double angle) const{ // keep angle within [-pi;pi)
        return std::fmod(angle + M_PI, (M_PI * 2)) - M_PI;
}

// Freq is 50Hz
void LoLoEKF::imuCB(const sensor_msgs::ImuPtr &imu_msg){
    using namespace boost::numeric::ublas;
    // TODO: implement noise-reduction filter here
    matrix<double> sigma_imu_hat;
    matrix<double> k_imu;
    matrix<double> Q_imu = identity_matrix<double>(3) * 0.001;
    matrix<double> S_invert = matrix<double>(3,3);
    matrix<double> I = identity_matrix<double>(3);

    vector<double> mu_hat_imu = mu_imu_;
    sigma_imu_hat = sigma_imu_;
    matrix<double> aux = (sigma_imu_hat + Q_imu);
    bool inverted = matrices::InvertMatrix(aux, S_invert);

    k_imu = prod(sigma_imu_hat, S_invert);
    vector<double> aux2 = vector<double>(3);
    aux2(0) = imu_msg->angular_velocity.x - mu_hat_imu(0);
    aux2(1) = imu_msg->angular_velocity.y - mu_hat_imu(1);
    aux2(2) = imu_msg->angular_velocity.z - mu_hat_imu(2);
    mu_imu_ = mu_hat_imu + prod(k_imu, aux2);
    sigma_imu_ = prod((I - k_imu), sigma_imu_hat);

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

void LoLoEKF::computeOdom(const sensor_msgs::ImuPtr sens_reading){
    double x_acc = sens_reading->linear_acceleration.x;
    double y_acc = sens_reading->linear_acceleration.y;
    double yaw_vel = sens_reading->angular_velocity.z;

    double t_now = sens_reading->header.stamp.toSec();
    delta_t_ = t_now - this->t_prev_;
    std::cout << "delta_t" << delta_t_ << std::endl;
    double delta_theta = angleLimit(yaw_vel * delta_t_);

    this->u_t_(0) = (mu_(2) * std::cos(mu_(4)) - mu_(1) * std::sin(mu_(4))) * delta_t_ +
            0.5 * (x_acc * std::cos(mu_(4)) - y_acc * std::sin(mu_(4))) * std::pow(delta_t_,2) -
            (mu_(3)/yaw_vel) * (std::cos(mu_(4) + delta_theta) - std::cos(mu_(4)));

    this->u_t_(1) = (mu_(2) * std::sin(mu_(4)) + mu_(1) * std::cos(mu_(4))) * delta_t_ +
            0.5 * (x_acc * std::sin(mu_(4)) + y_acc * std::cos(mu_(4))) * std::pow(delta_t_,2) +
            (mu_(3)/yaw_vel) * (std::sin(mu_(4) + delta_theta) - std::sin(mu_(4)));

    this->u_t_(2) = x_acc * delta_t_;
    this->u_t_(3) = y_acc * delta_t_;
    this->u_t_(4) = angleLimit(yaw_vel * delta_t_);

    this->t_prev_ = t_now;
}

void LoLoEKF::predictionStep(const sensor_msgs::ImuPtr sens_reading){
    using namespace boost::numeric::ublas;

    // Compute Jacobian of motion model for time t
    matrix<double> G_t = identity_matrix<double>(this->size_n_, this->size_n_);
    G_t(0,2) = std::cos(this->mu_(4)) *  delta_t_;
    G_t(0,3) = (1/sens_reading->angular_velocity.z) * (std::cos(mu_(4) + sens_reading->angular_velocity.z * delta_t_) - std::cos(mu_(4)))
                - std::sin(mu_(4) * delta_t_);
    G_t(0,4) = (mu_(3)/sens_reading->angular_velocity.z) * (std::sin(mu_(4)) - std::sin(mu_(4) + sens_reading->angular_velocity.z * delta_t_))
                - (mu_(2) * std::sin(mu_(4)) + mu_(3) * std::cos(mu_(4))) * delta_t_
                - 0.5 * (sens_reading->linear_acceleration.x * std::sin(mu_(4)) - sens_reading->linear_acceleration.y * std::cos(mu_(4))) * std::pow(delta_t_, 2);

    G_t(1,2) = std::sin(this->mu_(4)) *  delta_t_;
    G_t(1,3) = (1/sens_reading->angular_velocity.z) * (std::sin(mu_(4) + sens_reading->angular_velocity.z * delta_t_) - std::sin(mu_(4)))
                - std::cos(mu_(4) * delta_t_);
    G_t(1,4) = (mu_(3)/sens_reading->angular_velocity.z) * (std::cos(mu_(4)) - std::cos(mu_(4) + sens_reading->angular_velocity.z * delta_t_))
                - (mu_(2) * std::cos(mu_(4)) + mu_(3) * std::sin(mu_(4))) * delta_t_
                - 0.5 * (sens_reading->linear_acceleration.x * std::cos(mu_(4)) - sens_reading->linear_acceleration.y * std::sin(mu_(4))) * std::pow(delta_t_, 2);

    // Update prediction
    matrix<double> G_t_trans = trans(G_t);
    matrix<double> aux = prod(sigma_, G_t_trans);
    mu_hat_ = mu_ + u_t_;
    mu_hat_(4) = angleLimit(mu_hat_(4));
    sigma_hat_ = prod(G_t, aux) + R_;

    std::cout << "MU_" << mu_ << std::endl;

}

void LoLoEKF::predictMeasurementModel(){
    boost::numeric::ublas::vector<double> z_hat (size_k_);
    boost::numeric::ublas::matrix<double> identity = boost::numeric::ublas::identity_matrix<double>(size_n_);
}

void LoLoEKF::dataAssociation(){

}

void LoLoEKF::sequentialUpdate(){
    mu_ = mu_hat_;
    sigma_ = sigma_hat_;

}

void LoLoEKF::batchUpdate(){

}

void LoLoEKF::ekfLocalize(){
    boost::numeric::ublas::vector<double> mu_real = boost::numeric::ublas::vector<double>(3);
    ros::Rate rate(20);
    sensor_msgs::ImuPtr sensor_in;

    nav_msgs::Odometry odom_msg;
    tf::Quaternion q;

    ROS_INFO("Initialized");
    ROS_INFO("-------------------------");
    int cnt = 0;
    bool loop = true;
    bool init_filter = true;
    while(ros::ok()&& loop){
        ros::spinOnce();
        if(!imu_readings_.empty() && !gt_readings_.empty()){
//            ROS_DEBUG("*************************");
//            ROS_DEBUG("Sensors readings received");
//            ROS_DEBUG("*************************");
//            // Thread safe handling of sensor msgs queue **unnecessary**
//            {
//                boost::mutex::scoped_lock(msg_lock_);
//                sensor_in = imu_readings_.front();
//            }
//            // Initial estimate for the filter from ground truth
//            if(init_filter == true){
//                ROS_INFO("Filter initialized");
//                mu_(0) = gt_readings_.front()->pose.pose.position.x;
//                mu_(1) = gt_readings_.front()->pose.pose.position.y;
//                mu_(2) = gt_readings_.front()->twist.twist.linear.x;
//                mu_(3) = gt_readings_.front()->twist.twist.linear.y;
//                mu_(4) = tf::getYaw(gt_readings_.front()->pose.pose.orientation);
//                sigma_ = boost::numeric::ublas::identity_matrix<double>(this->size_n_) * 0.0000001;
//                init_filter = false;
//            }

//            ROS_DEBUG("----Computing odometry----");
//            computeOdom(sensor_in);
//            ROS_DEBUG("----Prediction step----");
//            predictionStep(sensor_in);
//            ROS_DEBUG("----Data association----");
//            dataAssociation();
//            ROS_DEBUG("----Sequential update----");
//            // Sequential or batch update
//            if(sequential_update_ == true){
//                sequentialUpdate();
//            }
//            else{
//                batchUpdate();
//            }

//            // Advertise results
//            odom_msg.header.stamp = sensor_in->header.stamp;
//            odom_msg.header.frame_id = "world";
//            odom_msg.child_frame_id = "lolo_auv/base_link";
//            odom_msg.pose.pose.position.x = mu_(0);
//            odom_msg.pose.pose.position.y = mu_(1);
//            odom_msg.pose.pose.position.z = gt_readings_.front()->pose.pose.position.z;
//            odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mu_(4));
//            odom_pub_.publish(odom_msg);

//            // Thread safe dispossal of sensor msgs
//            {
//                boost::mutex::scoped_lock(msg_lock_);
//                imu_readings_.pop();
//            }
//            gt_readings_.pop();
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
