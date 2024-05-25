#include "auv_graph_localization/auv_graph_localization.hpp"

GraphLocalization::GraphLocalization(ros::NodeHandle &nh, ros::NodeHandle &nh_stim, ros::NodeHandle &nh_gps) : 
                                                            nh_(&nh), nh_stim_(&nh_stim), nh_gps_(&nh_gps)
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
    nh_->param<std::string>(("odom_frame"), odom_frame_, "sam/odom");
    nh_->param<std::string>(("base_frame"), base_frame_, "sam/base_link");
    nh_->param<std::string>(("map_frame"), map_frame_, "map");
    nh_->param<std::string>(("utm_frame"), utm_frame_, "utm");

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

    nh_->param<std::string>(("path_top"), path_top, "/sam/dr/path");
    path_pub_ = nh_->advertise<nav_msgs::Path>(path_top, 1);

    nh_->param<std::string>(("preint_top"), preint_top, "/sam/dr/preint_pose");
    preint_pub_ = nh_->advertise<nav_msgs::Odometry>(preint_top, 1);

    nh_->param<std::string>(("odom_top"), odom_top, "/sam/dr/odom");
    odom_sub_ = nh_->subscribe(odom_top, 100, &GraphLocalization::OdomCb, this);

    aux_bool_ = false;
    aux_sub_ = nh_->subscribe("/aux", 1, &GraphLocalization::AuxCb, this);

    nh_->param<std::string>(("stim_top"), stim_top, "/sam/core/imu");
    // stim_sub_ = nh_stim->subscribe(stim_top, 1, &GraphLocalization::StimCb, this);

    nh_->param<std::string>(("gps_odom_top"), gps_top, "/sam/dr/gps_odom");
    gps_sub_ = nh_gps_->subscribe(gps_top, 1, &GraphLocalization::GpsCb, this);

    nh_->param<float>(("vis_rate"), vis_rate_, 1.);
    std::thread(&GraphLocalization::Visualize, this).detach();
    
    std::cout << "Graph node ready " << std::endl;
}

void GraphLocalization::AuxCb(const std_msgs::BoolConstPtr &aux_msg)
{
    aux_bool_ = aux_msg->data;
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
    // std::cout << "Odom cnt " << node_cnt_ << std::endl;
    depth_t_ = odom_msg->pose.pose.position.z;

    Vector3 ang_vel_t(odom_msg->twist.twist.angular.x,
                      odom_msg->twist.twist.angular.y,
                      odom_msg->twist.twist.angular.z);

    Vector3 lin_vel_t(odom_msg->twist.twist.linear.x, 
                      odom_msg->twist.twist.linear.y,
                      odom_msg->twist.twist.linear.z);

    // Copy node counter locally to fetch latest node
    Pose3 pose_latest;
    if (graph_->result_.exists(X(node_cnt_)))
    {
        pose_latest = graph_->result_.at<Pose3>(X(node_cnt_));
        std::cout << "Pre pose from result ===================" << std::endl;
    }
    else if (graph_->initial_estimate_.exists(X(node_cnt_)))
    {
        pose_latest = graph_->initial_estimate_.at<Pose3>(X(node_cnt_));
        std::cout << "Prev pose " << pose_latest.translation()[0] << ", " << pose_latest.translation()[1] << ", " << pose_latest.translation()[2] << std::endl;

        std::cout << "Pre pose from init " << std::endl;
    }
    else
    {
        std::cout << "Pre pose is zero (it should be integrating) " << std::endl;
    }

    node_cnt_ = node_cnt_ + 1;
    // std::cout << "Cnt in Odom cb " << node_cnt_ << std::endl;
    graph_->OdomNode(ang_vel_t, lin_vel_t, pose_latest, dt, node_cnt_, depth_t_);

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
            // Attempt deep copy of graph object for plotting
            // TODO: define clone() withing the graph class to use mutex while cloning
            // boost::shared_ptr<GraphND> graph_plot;
            // graph_plot = boost::make_shared<GraphND>(*graph_);

            nav_msgs::Path path;
            path.header.frame_id = odom_frame_;
            path.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped pose_msg;
            std::vector<double> pose_i;

            // Node_cnt doesn't not reflect the actual number of nodes in the graph, some of them 
            // might not have been added already because they mutex is being held 
            int graph_nodes = graph_->result_.size() + graph_->initial_estimate_.size();
            // std::cout << "Nodes to be plotted " << graph_nodes -1 << std::endl;
            // std::cout << "Nodes in cnt " << node_cnt_ + 1 << std::endl;

            // Publish path in rviz
            for (int i = 0; i < graph_nodes; i++)
            {
                if (graph_->result_.exists(X(i)))
                {
                    pose_i = graph_->getValue(graph_->result_, i);
                    // std::cout << "From result " << pose_i.at(0) << ", " << pose_i.at(1) << ", " << pose_i.at(2) << std::endl;
                }
                else if (graph_->initial_estimate_.exists(X(i)))
                {
                    pose_i = graph_->getValue(graph_->initial_estimate_, i);
                    // std::cout << "From init " << pose_i.at(0) << ", " << pose_i.at(1) << ", " << pose_i.at(2) << std::endl;
                }
                else
                {
                    ROS_WARN_STREAM("Graph loc: rviz thread might be out of synch");
                    continue;
                }

                pose_msg.pose.position.x = pose_i.at(0);
                pose_msg.pose.position.y = pose_i.at(1);
                pose_msg.pose.position.z = 0;
                pose_msg.pose.position.z = (pose_i.size() > 2)? pose_i.at(2): 0;

                pose_msg.pose.orientation.x = pose_i.at(3);
                pose_msg.pose.orientation.y = pose_i.at(4);
                pose_msg.pose.orientation.z = pose_i.at(5);
                pose_msg.pose.orientation.w = pose_i.at(6);
                path.poses.push_back(pose_msg);
            }
            path_pub_.publish(path);

            // BR odom-->base at time t with last pose_msg
            tf_odom_base_.header.frame_id = odom_frame_;
            tf_odom_base_.child_frame_id = base_frame_;
            tf_odom_base_.header.stamp = ros::Time::now();

            tf_odom_base_.transform.translation.x = pose_msg.pose.position.x;
            tf_odom_base_.transform.translation.y = pose_msg.pose.position.y;
            tf_odom_base_.transform.translation.z = pose_msg.pose.position.z;
            tf_odom_base_.transform.rotation.x = pose_msg.pose.orientation.x;
            tf_odom_base_.transform.rotation.y = pose_msg.pose.orientation.y;
            tf_odom_base_.transform.rotation.z = pose_msg.pose.orientation.z;
            tf_odom_base_.transform.rotation.w = pose_msg.pose.orientation.w;
            static_broadcaster_.sendTransform(tf_odom_base_);
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
    if(!aux_bool_)
    {
        int cnt = node_cnt_;
        std::cout << "Cnt in GPS cb " << node_cnt_ << std::endl;

        geometry_msgs::PoseStamped gps_utm, gps_odom;
        gps_utm.header.frame_id = utm_frame_;
        gps_utm.pose.position.x = gps_msg->pose.pose.position.x;
        gps_utm.pose.position.y = gps_msg->pose.pose.position.y;
        gps_utm.pose.position.z = 0.;
        tf2::doTransform(gps_utm, gps_odom, utm_odom_tf_);
        std::cout << "GPS fix " << gps_odom.pose.position.x << ", " << gps_odom.pose.position.y << ", " << gps_odom.pose.position.z << std::endl;
        std::vector<double> gps_vec{gps_odom.pose.position.x, gps_odom.pose.position.y};

        try
        {
            graph_->GpsNode(gps_vec, cnt, depth_t_);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM("Graph loc node. GPS fix: " << e.what());
        }
    }

    // try
    // {
    //     // Optimize here
    //     graph_->Optimize(cnt);
    // }
    // catch(const std::exception& e)
    // {
    //     ROS_WARN_STREAM("Graph loc node. Optimize step: " << e.what());
    // }
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