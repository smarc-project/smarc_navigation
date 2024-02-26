#include "auv_graph_localization/auv_graph_localization.hpp"

GraphLocalization::GraphLocalization(ros::NodeHandle &nh, ros::NodeHandle &nh_stim) : nh_(&nh), nh_stim_(&nh_stim)
{

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam2_ = new ISAM2(parameters);

    // Init prior
    node_cnt_ = 0;
    stim_cnt_ = 0;
    
    Vector10 initial_state;
    for (int i = 0; i < 9; i++)
    {
        initial_state(i) = 0.;
    }
    Rot3 prior_rotation = Rot3::Quaternion(1, 0,0,0);
    Point3 prior_point(0,0,0);
    Pose3 prior_pose(prior_rotation, prior_point);
    odom_pose_prev_ = prior_pose;
    Vector3 prior_velocity(initial_state.tail<3>());
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

    // Add all prior factors (pose, velocity, bias) to the graph.
    initial_estimate_.insert(X(node_cnt_), prior_pose);
    initial_estimate_.insert(V(node_cnt_), prior_velocity);
    initial_estimate_.insert(B(node_cnt_), prior_imu_bias);

    // Assemble prior noise model and add it the graph.`
    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5)
            .finished());                                             // rad,rad,rad,m, m, m
    auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1); // m/s

    bias_noise_model_ = noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior factors (pose, velocity, bias) to the graph.
    graph_ = new NonlinearFactorGraph();
    graph_->addPrior(X(node_cnt_), prior_pose, pose_noise_model);
    graph_->addPrior(V(node_cnt_), prior_velocity, velocity_noise_model);
    graph_->addPrior(B(node_cnt_), prior_imu_bias, bias_noise_model_);

    // STIM params and preintegrator
    p_ = this->stimParams();
    preintegrated_.reset(new PreintegratedImuMeasurements(p_, prior_imu_bias));
    
    // Store previous state for imu integration and latest predicted outcome.
    prev_state_ = new NavState(prior_pose, prior_velocity);
    prev_bias_ = prior_imu_bias;

    // STIM is not started
    stim_init_ = false;

    // Optimization has not started
    optimized_ = false;

    tf2_ros::TransformListener tf_listener(tf_buffer_);
    nh.param<std::string>(("odom_frame"), odom_frame_, "sam/odom");
    nh.param<std::string>(("map_frame"), map_frame_, "map");
    nh.param<std::string>(("utm_frame"), utm_frame_, "utm");

    try
    {
        ROS_DEBUG("Waiting for transforms");
        auto asynch_1 = std::async(std::launch::async, [this]
                                   { return tf_buffer_.lookupTransform(odom_frame_, utm_frame_,
                                                                       ros::Time(0), ros::Duration(60.)); });
        utm_odom_tf_ = asynch_1.get();
        std::cout << "UTM to odom " << utm_odom_tf_.transform.translation.x << ", " 
                                    << utm_odom_tf_.transform.translation.y << ", " 
                                    << utm_odom_tf_.transform.translation.z <<  std::endl;

        ROS_INFO("Graph loc node: utm to odom locked");
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("ERROR: Could not lookup transform from utm to odom");
    }


    std::string odom_top, stim_top, path_top, gps_top;
    nh.param<float>(("vis_rate"), vis_rate_, 1.);
    nh.param<std::string>(("odom_top"), odom_top, "/sam/dr/odom");
    odom_sub_ = nh.subscribe(odom_top, 1, &GraphLocalization::OdomCb, this);

    nh.param<std::string>(("stim_top"), stim_top, "/sam/core/imu");
    stim_sub_ = nh_stim.subscribe(stim_top, 1, &GraphLocalization::StimCb, this);

    nh.param<std::string>(("gps_top"), gps_top, "/sam/dr/gps_odom_fake");
    gps_sub_ = nh.subscribe(gps_top, 1, &GraphLocalization::GpsCb, this);

    nh.param<std::string>(("path_top"), path_top, "/sam/dr/path");
    path_pub_ = nh.advertise<nav_msgs::Path>(path_top, 1);


    std::thread(&GraphLocalization::Visualize, this).detach();

    std::cout << "Graph node ready " << std::endl;
}

void GraphLocalization::OdomCb(const nav_msgs::OdometryConstPtr &odom_msg)
{
    if (stim_init_)
    {
        node_cnt_ = node_cnt_ + 1;
        std::cout << "Odom cnt " << node_cnt_ << std::endl;
        
        // Add odometry estimate
        Rot3 odom_rotation = Rot3::Quaternion(odom_msg->pose.pose.orientation.w,
                                              odom_msg->pose.pose.orientation.x,
                                              odom_msg->pose.pose.orientation.y,
                                              odom_msg->pose.pose.orientation.z);
        Point3 odom_point(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
        Pose3 odom_pose(odom_rotation, odom_point);
        Vector3 odom_velocity(odom_msg->twist.twist.angular.x, odom_msg->twist.twist.angular.y, odom_msg->twist.twist.angular.z);
        gtsam::NavState odom_estimate(odom_pose, odom_velocity);
        initial_estimate_.insert(X(node_cnt_), odom_estimate.pose());
        // initial_estimate_.insert(V(node_cnt_), odom_estimate.v());
        // initial_estimate_.insert(B(node_cnt_), prev_bias_);

        // Add odometry factors between consecutive poses
        Pose3 odom_step = odom_pose.compose(odom_pose_prev_.inverse());
        // TODO: extract noise from odom_msg
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished());
        graph_->add(BetweenFactor<Pose3>(X(node_cnt_ - 1), X(node_cnt_), odom_step, odometryNoise));

        // Add IMU factor factor
        // auto preint_imu = dynamic_cast<const PreintegratedImuMeasurements &>(*preintegrated_);
        // gtsam::ImuFactor imu_factor(X(node_cnt_ - 1), V(node_cnt_ - 1),
        //                             X(node_cnt_), V(node_cnt_),
        //                             B(node_cnt_ - 1), preint_imu);
        // graph_->add(imu_factor);
        // imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        // graph_->add(BetweenFactor<imuBias::ConstantBias>(
        //     B(node_cnt_ - 1), B(node_cnt_), zero_bias,
        //     bias_noise_model_));


        // Nacho: For testing only
        // NavState prop_state = preintegrated_->predict(*prev_state_, prev_bias_);

        odom_pose_prev_ = odom_pose;
    }
}

void GraphLocalization::Optimize()
{
    // Optimize
    isam2_->update(*graph_, initial_estimate_);
    result_ = isam2_->calculateEstimate();
    std::cout << " -----------------Result " << result_.size() << std::endl;

    // Reset the graph
    graph_->resize(0);
    initial_estimate_.clear();
    optimized_ = true;

    // Overwrite the beginning of the preintegration for the next step.
    // prev_state_ = new NavState(result_.at<Pose3>(X(node_cnt_)),
    //                            result_.at<Vector3>(V(node_cnt_)));
    // prev_bias_ = result_.at<imuBias::ConstantBias>(B(node_cnt_));

    // // TODO publish result here to vis in rviz
    // Vector3 gtsam_position = prev_state_->pose().translation();
    // Vector3 imu_position = prop_state.pose().translation();
    // cout << "GTSAM position:" << gtsam_position(0) << ", " << gtsam_position(1) << ", " << gtsam_position(2) << "\t "
    //      << "\n";
    // cout << "STIM position:" << imu_position(0) << ", " << imu_position(1) << ", " << imu_position(2) << "\t "
    //      << "\n";

    // Reset the preintegration object.
    preintegrated_->resetIntegrationAndSetBias(prev_bias_);
}

void GraphLocalization::Visualize()
{
    ros::Rate r(vis_rate_);
    while(ros::ok())
    {
        std::cout << "Plotting " << std::endl;

        nav_msgs::Path path;
        path.header.frame_id = odom_frame_;
        path.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped pose_msg;
        Pose3 pose_i;

        for(int i = 0; i <= node_cnt_; i++)
        {
            if (result_.exists(X(i)))
            {
                pose_i = result_.at<Pose3>(X(i));
            }
            else if (initial_estimate_.exists(X(i)))
            {
                pose_i = initial_estimate_.at<Pose3>(X(i));
            }

            pose_msg.pose.position.x = pose_i.translation()[0];
            pose_msg.pose.position.y = pose_i.translation()[1];
            pose_msg.pose.position.z = pose_i.translation()[2];
            pose_msg.pose.orientation.w = 1;
            path.poses.push_back(pose_msg);
        }

        path_pub_.publish(path);
        r.sleep();
    }
}


void GraphLocalization::StimCb(const sensor_msgs::ImuConstPtr &imu_msg)
{
    stim_t_now_ = imu_msg->header.stamp.toSec();
    if (!stim_init_)
    {
        stim_t_prev_ = stim_t_now_;
        stim_init_ = true;
        return;
    }
    stim_cnt_ = stim_cnt_ + 1;
    // std::cout << "Stim cnt " << stim_cnt_ << std::endl;

    // Adding the IMU preintegration.
    double dt = stim_t_now_ - stim_t_prev_;

    // TODO: not integrating accelerations for now
    preintegrated_->integrateMeasurement(Vector3(0., 0., 0.), 
                                        Vector3(imu_msg->angular_velocity.x, 
                                                imu_msg->angular_velocity.y, 
                                                imu_msg->angular_velocity.z), dt);
}


void GraphLocalization::GpsCb(const nav_msgs::OdometryConstPtr &gps_msg)
{
    try
    {
        std::cout << "GPS meas " << std::endl;
        // tf_buffer_.lookupTransform(map_frame_, odom_frame_, ros::Time(0), ros::Duration(1.));
        // geometry_msgs::TransformStamped utm_to_odom = tf_buffer_.lookupTransform(utm_frame_, odom_frame_, ros::Time(0), ros::Duration(1.));
        geometry_msgs::PoseStamped gps_utm, gps_odom;
        gps_utm.header.frame_id = utm_frame_;
        gps_utm.pose.position.x = gps_msg->pose.pose.position.x;
        gps_utm.pose.position.y = gps_msg->pose.pose.position.y;
        gps_utm.pose.position.z = 0.;
        tf2::doTransform(gps_utm, gps_odom, utm_odom_tf_);

        std::cout << "GPS fix " << gps_odom.pose.position.x << ", " << gps_odom.pose.position.y << ", " << gps_odom.pose.position.z << std::endl;

        auto correction_noise = noiseModel::Isotropic::Sigma(3, 1.0);
        GPSFactor gps_factor(X(node_cnt_),
                             Point3(gps_odom.pose.position.x,  // N,
                                    gps_odom.pose.position.y,  // E,
                                    gps_odom.pose.position.z), // D,
                             correction_noise);
        graph_->add(gps_factor);

        // Optimize here
        this->Optimize();
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM("Graph loc node: Could not lookup transform from " << utm_frame_ << " to " << odom_frame_);
    }
}

boost::shared_ptr<PreintegratedCombinedMeasurements::Params> GraphLocalization::stimParams()
{
    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma = 0.0003924;
    double gyro_noise_sigma = 0.000205689024915;
    double accel_bias_rw_sigma = 0.004905;
    double gyro_bias_rw_sigma = 0.000001454441043;
    Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
    Matrix33 integration_error_cov =
        I_3x3 * 1e-8; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
    Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
    Matrix66 bias_acc_omega_init =
        I_6x6 * 1e-5; // error in the bias used for preintegration

    auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    // PreintegrationBase params:
    p->accelerometerCovariance =
        measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance =
        integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance =
        measured_omega_cov; // gyro white noise in continuous
    // PreintegrationCombinedMeasurements params:
    p->biasAccCovariance = bias_acc_cov;     // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_init;

    return p;
}