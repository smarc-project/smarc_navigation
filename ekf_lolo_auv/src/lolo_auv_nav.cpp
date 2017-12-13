#include "lolo_auv_nav/lolo_auv_nav.hpp"


LoLoEKF::LoLoEKF(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh){

    std::string imu_topic;
    std::string dvl_topic;
    std::string odom_topic;
    std::string gt_topic;
    std::string rpt_topic;

    nh_->param<std::string>((node_name_ + "/imu_topic"), imu_topic, "/imu");
    nh_->param<std::string>((node_name_ + "/dvl_topic"), dvl_topic, "/dvl");
    nh_->param<std::string>((node_name_ + "/odom_pub_topic"), odom_topic, "/odom_ekf");
    nh_->param<std::string>((node_name_ + "/gt_pose_topic"), gt_topic, "/gt_pose");
    nh_->param<std::string>((node_name_ + "/rpt_topic"), rpt_topic, "/rpt_topic");
    nh_->param<std::string>((node_name_ + "/map_srv"), map_srv_name_, "/get_map");
    nh_->param<std::string>((node_name_ + "/odom_frame"), odom_frame_, "/odom");
    nh_->param<std::string>((node_name_ + "/world_frame"), world_frame_, "/world");
    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/dvl_frame"), dvl_frame_, "/dvl_link");

    // Synch IMU and DVL readings
    imu_subs_ = new message_filters::Subscriber<sensor_msgs::Imu>(*nh_, imu_topic, 50);
    dvl_subs_ = new message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>(*nh_, dvl_topic, 10);
    msg_synch_ptr_ = new message_filters::Synchronizer<MsgTimingPolicy> (MsgTimingPolicy(100), *imu_subs_, *dvl_subs_);
    msg_synch_ptr_->registerCallback(boost::bind(&LoLoEKF::synchSensorsCB, this, _1, _2));

    rpt_subs_ = nh_->subscribe(rpt_topic, 10, &LoLoEKF::rptCB, this);
    tf_gt_subs_ = nh_->subscribe(gt_topic, 10, &LoLoEKF::gtCB, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic, 10);
    map_client_ = nh_->serviceClient<ekf_lolo_auv::map_ekf>(map_srv_name_);
    vis_pub_ = nh_->advertise<visualization_msgs::MarkerArray>( "/lolo_auv/landmarks", 0 );

    // Initialize attributes
    init();
}

void LoLoEKF::synchSensorsCB(const sensor_msgs::ImuConstPtr &imu_msg, const geometry_msgs::TwistWithCovarianceStampedConstPtr &dvl_msg){
    imu_readings_.push(imu_msg);
    dvl_readings_.push(dvl_msg);
}

//auto timerCallback = [](auto input) { ROS_ERROR("Map server not available"); };

void LoLoEKF::init(){

    // Get map from provider
    while(!ros::service::waitForService(map_srv_name_, ros::Duration(10)) && ros::ok()){
        ROS_INFO_NAMED(node_name_,"Waiting for the map server to come up");
    }

    ekf_lolo_auv::map_ekf map_req;
    map_req.request.request_map = true;
    if(map_client_.call(map_req)){
        boost::numeric::ublas::vector<double> aux_vec(3);
        for (auto landmark: map_req.response.map){
            aux_vec(0) = landmark.x;
            aux_vec(1) = landmark.y;
            aux_vec(2) = landmark.z;
            map_.push_back(aux_vec);
        }
    }
    createMapMarkers();
    ROS_INFO("Initialized");
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

void LoLoEKF::gtCB(const nav_msgs::OdometryPtr &pose_msg){
    gt_readings_.push(pose_msg);
}


void LoLoEKF::rptCB(const geometry_msgs::PoseWithCovarianceStampedPtr &ptr_msg){

}

void LoLoEKF::createMapMarkers(){

    unsigned int i = 0;
    for (auto landmark: map_){
        visualization_msgs::Marker markers;
        markers.header.frame_id = "world";
        markers.header.stamp = ros::Time();
        markers.ns = "lolo_auv";
        markers.id = i;
        markers.type = visualization_msgs::Marker::CUBE;
        markers.action = visualization_msgs::Marker::ADD;
        markers.pose.position.x = landmark(0);
        markers.pose.position.y = landmark(1);
        markers.pose.position.z = landmark(2);
        markers.pose.orientation.x = 0.0;
        markers.pose.orientation.y = 0.0;
        markers.pose.orientation.z = 0.0;
        markers.pose.orientation.w = 1.0;
        markers.scale.x = 1;
        markers.scale.y = 1;
        markers.scale.z = 1;
        markers.color.a = 1.0; // Don't forget to set the alpha!
        markers.color.r = 0.0;
        markers.color.g = 1.0;
        markers.color.b = 0.0;

        markers_.markers.push_back(markers);
        i += 1;
    }
    std::cout << "number of landmars: " << i << std::endl;
}

bool LoLoEKF::sendOutput(ros::Time t){

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

void LoLoEKF::computeOdom(const geometry_msgs::TwistWithCovarianceStampedConstPtr &dvl_msg, const tf::Quaternion q_auv,
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
    mu_(2) = angleLimit(mu_(2));
    Sigma_ = Sigma_hat_;
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

    sensor_msgs::ImuConstPtr imu_msg;
    geometry_msgs::TwistWithCovarianceStampedConstPtr dvl_msg;
    nav_msgs::OdometryPtr gt_msg;
    double t_prev;
    tf::Quaternion q_auv;

    // TODO: full implementation of 6 DOF movement

    boost::numeric::ublas::vector<double> u_t = boost::numeric::ublas::vector<double>(3);

    bool loop = true;
    bool init_filter = true;

    while(ros::ok()&& loop){
        ros::spinOnce();
//        std::cout << "Sizes: " <<  dvl_readings_.size() << ", " << imu_readings_.size()<< std::endl;

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

            // Fetch latest sensor readings
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

            // Publish and broadcast
            this->sendOutput(dvl_msg->header.stamp);

            vis_pub_.publish(markers_);

            imu_readings_.pop();
            dvl_readings_.pop();
            gt_readings_.pop();
        }
        else{
            ROS_DEBUG("Still waiting for some good, nice sensor readings...");
        }
        rate.sleep();
    }
}

LoLoEKF::~LoLoEKF(){
    // TODO_NACHO: do some cleaning here
}
