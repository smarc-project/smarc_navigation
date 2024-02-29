#include <auv_graph_localization/graph.hpp>

GraphND::GraphND(int &node_cnt)
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam2_ = new ISAM2(parameters);
    std::cout << "Graph object constructed" << std::endl;
}

Graph2D::Graph2D(int& node_cnt): GraphND(node_cnt)
{
    // 2D prior
    Pose2 prior_pose(0.0, 0.0, 0.0); // prior at origin
    initial_estimate_.insert(X(node_cnt), prior_pose);

    // Assemble prior noise model and add it the graph.`
    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(3) << 0.01, 0.01, 0.01)
            .finished()); // rad,rad,rad,m, m, m

    graph_ = new NonlinearFactorGraph();
    graph_->add(PriorFactor<Pose2>(X(node_cnt), prior_pose, pose_noise_model));
}

Graph3D::Graph3D(int &node_cnt): GraphND(node_cnt)
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam2_ = new ISAM2(parameters);

    // 3D prior
    Rot3 prior_rotation = Rot3::Quaternion(1, 0,0,0);
    Point3 prior_point(0,0,0);
    Pose3 prior_pose(prior_rotation, prior_point);
    initial_estimate_.insert(X(node_cnt), prior_pose);

    // Vector3 prior_velocity(0,0,0);
    // imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
    // p_ = this->stimParams();
    // preintegrated_.reset(new PreintegratedImuMeasurements(p_, prior_imu_bias));
    // prop_state_ = NavState(prior_pose, prior_velocity);
    // initial_estimate_.insert(V(node_cnt), prior_velocity);
    // initial_estimate_.insert(B(node_cnt), prior_imu_bias);


    // auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1); // m/s
    // bias_noise_model_ = noiseModel::Isotropic::Sigma(6, 1e-3);

    // Assemble prior noise model and add it the graph.`
    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)
            .finished()); // rad,rad,rad,m, m, m

    graph_ = new NonlinearFactorGraph();
    graph_->add(PriorFactor<Pose3>(X(node_cnt), prior_pose, pose_noise_model));
    // graph_->addPrior(V(node_cnt_), prior_velocity, velocity_noise_model);
    // graph_->addPrior(B(node_cnt_), prior_imu_bias, bias_noise_model_);
}

void Graph2D::OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, double dt, int &node_cnt, double depth)
{
    // depth_t_ = odom_msg->pose.pose.position.z;

    Vector3 euler = odom_rotation.rpy();

    Vector3 odom_step = odom_rotation.matrix() * lin_vel_t * dt;
    Pose2 odom_pose(odom_pose_prev_.translation()[0] + odom_step[0],
                    odom_pose_prev_.translation()[1] + odom_step[1],
                    euler[2]);

    initial_estimate_.insert(X(node_cnt), odom_pose);

    // Add odometry factors between consecutive poses
    // Below is equivalent to odom_pose_prev_.between(odom_pose)
    // Pose2 odom_step = odom_pose_prev_.inverse().compose(odom_pose);
    // TODO: extract noise from odom_msg
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 1., 1., 0.1).finished());
    graph_->add(BetweenFactor<Pose2>(X(node_cnt - 1), X(node_cnt), odom_pose_prev_.between(odom_pose), odometryNoise));

    odom_pose_prev_ = odom_pose;
}

void Graph3D::OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, double dt, int &node_cnt, double depth)
{

    Vector3 odom_step = odom_rotation.matrix() * lin_vel_t * dt;
    Point3 odom_position(odom_pose_prev_.translation()[0] + odom_step[0],
                         odom_pose_prev_.translation()[1] + odom_step[1],
                         depth);
    Pose3 odom_pose(odom_rotation, odom_position);

    // Vector3 odom_velocity(odom_msg->twist.twist.angular.x, odom_msg->twist.twist.angular.y, odom_msg->twist.twist.angular.z);
    // gtsam::NavState odom_estimate(odom_pose, odom_velocity);
    initial_estimate_.insert(X(node_cnt), odom_pose);
    // initial_estimate_.insert(V(node_cnt_), odom_estimate.v());
    // initial_estimate_.insert(B(node_cnt_), prev_bias_);

    // Add odometry factors between consecutive poses
    // Below is equivalent to odom_pose_prev_.between(odom_pose)
    // Pose2 odom_step = odom_pose_prev_.inverse().compose(odom_pose);
    // TODO: extract noise from odom_msg
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 1., 1., 0.1, 0.1, 0.1, 0.1).finished());
    graph_->add(BetweenFactor<Pose3>(X(node_cnt - 1), X(node_cnt), odom_pose_prev_.between(odom_pose), odometryNoise));

    odom_pose_prev_ = odom_pose;

    // Pitch constraint for 3D case
    // noiseModel::Diagonal::shared_ptr pitchNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 0.0001).finished());
    // Pose3PitchFactor pitch_factor(X(node_cnt_), euler[1], pitchNoise);
    // graph_->add(pitch_factor);

    // prop_state_ = preintegrated_->predict(prop_state_, prev_bias_);

    // }
}


void Graph3D::DepthPrior(int cnt, double depth)
{
    // Depth constraint for 3D case
    noiseModel::Diagonal::shared_ptr depthNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 0.01).finished());
    // Pose3DepthFactor depth_factor(X(cnt), depth, depthNoise);
    graph_->add(boost::make_shared<Pose3DepthFactor>(X(cnt), depth, depthNoise));
    // graph_->add(depth_factor);
}

void Graph2D::GpsNode(const std::vector<double> &gps_odom, int &node_cnt)
{
    // 2D version
    auto unaryNoise = noiseModel::Isotropic::Sigma(2, 10.0);
    graph_->add(boost::make_shared<UnaryFactor>(X(node_cnt), gps_odom.at(0), gps_odom.at(1), unaryNoise));

    // 3D version
    // auto correction_noise = noiseModel::Isotropic::Sigmas((Vector(3) << 10., 10., 0.1).finished());
    // GPSFactor gps_factor(X(cnt),
    //                      Point3(gps_odom.pose.position.x, // N,
    //                             gps_odom.pose.position.y, // E,
    //                             depth_t_),                // D,
    //                      correction_noise);
    // graph_->add(gps_factor);

    // noiseModel::Diagonal::shared_ptr depthNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 0.001).finished());
    // Pose3DepthFactor depth_factor(X(cnt), depth_t_, depthNoise);
    // graph_->add(depth_factor);
}

void Graph3D::GpsNode(const std::vector<double> &gps_odom, int &node_cnt)
{
    // 3D version
    auto correction_noise = noiseModel::Isotropic::Sigmas((Vector(3) << 10., 10., 0.1).finished());
    GPSFactor gps_factor(X(node_cnt),
                         Point3(gps_odom.at(0), // N,
                                gps_odom.at(1), // E,
                                0.),                // D,
                         correction_noise);
    graph_->add(gps_factor);

    // noiseModel::Diagonal::shared_ptr depthNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 0.001).finished());
    // Pose3DepthFactor depth_factor(X(cnt), depth_t_, depthNoise);
    // graph_->add(depth_factor);
}

void Graph2D::Optimize(int cnt)
{
    // Optimize

    // iSAM2
    std::cout << "----------------- Optimizing --------------------" << std::endl;
    // writeG2o(*graph_, initial_estimate_, "before.dot");
    ISAM2Result update_info = isam2_->update(*graph_, initial_estimate_);
    update_info.print();

    // Update odom estimate
    result_ = isam2_->calculateEstimate();
    odom_pose_prev_ = result_.at<Pose2>(X(cnt));

    // Reset the graph
    graph_->resize(0);
    initial_estimate_.clear();

    // Reset the preintegration object.
    // preintegrated_->resetIntegrationAndSetBias(prev_bias_);
}

void Graph3D::Optimize(int cnt)
{
    // Optimize

    // iSAM2
    std::cout << "----------------- Optimizing --------------------" << std::endl;
    // writeG2o(*graph_, initial_estimate_, "before.dot");
    ISAM2Result update_info = isam2_->update(*graph_, initial_estimate_);
    update_info.print();

    // Update odom estimate
    result_ = isam2_->calculateEstimate();
    odom_pose_prev_ = result_.at<Pose3>(X(cnt));

    // Reset the graph
    graph_->resize(0);
    initial_estimate_.clear();

    // Reset the preintegration object.
    // preintegrated_->resetIntegrationAndSetBias(prev_bias_);
}

std::vector<double> Graph2D::getValue(Values& values, int i)
{
    Pose2 pose_i = values.at<Pose2>(X(i));

    return std::vector<double>{pose_i.translation()[0], pose_i.translation()[1]};
}

std::vector<double> Graph3D::getValue(Values &values, int i)
{
    Pose3 pose_i = values.at<Pose3>(X(i));

    return std::vector<double>{pose_i.translation()[0], pose_i.translation()[1], pose_i.translation()[2]};
}