#include "auv_graph_localization/auv_graph_localization.hpp"

GraphLocalization::GraphLocalization(ros::NodeHandle &nh, ros::NodeHandle &nh_stim) : nh_(&nh), nh_stim_(&nh_stim)
{

    // Init prior
    node_cnt_ = 0;
    stim_cnt_ = 0;
    
    // STIM is not started
    stim_init_ = false;
    odom_init_ = false;
    depth_t_ = 0.;

    graph_.reset(new Graph3D(node_cnt_));

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

    std::string odom_top, stim_top, path_top, gps_top, preint_top;
    nh.param<float>(("vis_rate"), vis_rate_, 1.);
    nh.param<std::string>(("odom_top"), odom_top, "/sam/dr/odom");
    odom_sub_ = nh.subscribe(odom_top, 100, &GraphLocalization::OdomCb, this);

    nh.param<std::string>(("stim_top"), stim_top, "/sam/core/imu");
    // stim_sub_ = nh_stim.subscribe(stim_top, 1, &GraphLocalization::StimCb, this);

    nh.param<std::string>(("gps_top"), gps_top, "/sam/dr/gps_odom_fake");
    gps_sub_ = nh.subscribe(gps_top, 1, &GraphLocalization::GpsCb, this);

    nh.param<std::string>(("path_top"), path_top, "/sam/dr/path");
    path_pub_ = nh.advertise<nav_msgs::Path>(path_top, 1);

    nh.param<std::string>(("preint_top"), preint_top, "/sam/dr/preint_pose");
    preint_pub_ = nh.advertise<nav_msgs::Odometry>(preint_top, 1);

    std::thread(&GraphLocalization::Visualize, this).detach();

    std::cout << "Graph node ready " << std::endl;
}


void GraphLocalization::OdomCb(const nav_msgs::OdometryConstPtr &odom_msg)
{
    odom_t_now_ = odom_msg->header.stamp.toSec();
    if (!odom_init_)
    {
        odom_t_prev_ = odom_t_now_;
        odom_init_ = true;
        return;
    }
    double dt = odom_t_now_ - odom_t_prev_;

    // if (stim_init_)
    // {
    node_cnt_ = node_cnt_ + 1;
    std::cout << "Odom cnt " << node_cnt_ << std::endl;
    depth_t_ = odom_msg->pose.pose.position.z;

    Rot3 odom_rotation = Rot3::Quaternion(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z);
    Vector3 lin_vel_t(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);

    graph_->OdomNode(odom_rotation, lin_vel_t, dt, node_cnt_, depth_t_);

    // Add a depth prior every x nodes. It will not do anything if the graph is 2D
    // if (node_cnt_ % 100 == 0)
    // {
        graph_->DepthPrior(node_cnt_, depth_t_);
    // }

    odom_t_prev_ = odom_t_now_;
}

void GraphLocalization::Visualize()
{
    ros::Rate r(vis_rate_);
    while(ros::ok())
    {
        if(node_cnt_ > 2)
        {
            std::cout << "Plotting " << std::endl;

            nav_msgs::Path path;
            path.header.frame_id = odom_frame_;
            path.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped pose_msg;
            std::vector<double> pose_i;

            for(int i = 0; i <= node_cnt_; i++)
            {
                if (graph_->result_.exists(X(i)))
                {
                    pose_i = graph_->getValue(graph_->result_, i);
                }
                else if (graph_->initial_estimate_.exists(X(i)))
                {
                    pose_i = graph_->getValue(graph_->initial_estimate_, i);
                }

                pose_msg.pose.position.x = pose_i.at(0);
                pose_msg.pose.position.y = pose_i.at(1);
                pose_msg.pose.position.z = 0;
                pose_msg.pose.position.z = (pose_i.size() > 2)? pose_i.at(2): 0;

                // TODO: add orientation
                pose_msg.pose.orientation.w = 1;
                path.poses.push_back(pose_msg);
            }
            path_pub_.publish(path);
        }

        // nav_msgs::Odometry preint_odom;
        // preint_odom.header.frame_id = odom_frame_;
        // preint_odom.header.stamp = ros::Time::now();
        // preint_odom.pose.pose.position.x = prop_state_.pose().translation()[0];
        // preint_odom.pose.pose.position.y = prop_state_.pose().translation()[1];
        // preint_odom.pose.pose.position.z = prop_state_.pose().translation()[2];
        // preint_odom.pose.pose.orientation.w = prop_state_.pose().rotation().quaternion()[0];
        // preint_odom.pose.pose.orientation.x = prop_state_.pose().rotation().quaternion()[1];
        // preint_odom.pose.pose.orientation.y = prop_state_.pose().rotation().quaternion()[2];
        // preint_odom.pose.pose.orientation.z = prop_state_.pose().rotation().quaternion()[3];
        // preint_pub_.publish(preint_odom);

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

    // // TODO: not integrating accelerations for now
    // preintegrated_->integrateMeasurement(Vector3(0., 0., 0.), 
    //                                     Vector3(imu_msg->angular_velocity.x, 
    //                                             imu_msg->angular_velocity.y, 
    //                                             imu_msg->angular_velocity.z), dt);
}


void GraphLocalization::GpsCb(const nav_msgs::OdometryConstPtr &gps_msg)
{
    try
    {
        int cnt = node_cnt_;

        geometry_msgs::PoseStamped gps_utm, gps_odom;
        gps_utm.header.frame_id = utm_frame_;
        gps_utm.pose.position.x = gps_msg->pose.pose.position.x;
        gps_utm.pose.position.y = gps_msg->pose.pose.position.y;
        gps_utm.pose.position.z = 0.;
        tf2::doTransform(gps_utm, gps_odom, utm_odom_tf_);
        std::cout << "GPS fix " << gps_odom.pose.position.x << ", " << gps_odom.pose.position.y << ", " << gps_odom.pose.position.z << std::endl;

        std::vector<double> gps_vec{gps_odom.pose.position.x, gps_odom.pose.position.y};
        graph_->GpsNode(gps_vec, cnt);

        // // Add depth prior. It will not do anything if graph is 2D
        // graph_->DepthPrior(node_cnt_, depth_t_);

        // Optimize here
        graph_->Optimize(cnt);
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