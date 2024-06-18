#include <auv_graph_localization/graph.hpp>

GraphND::GraphND(){}

GraphND::GraphND(int &node_cnt)
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam2_ = new ISAM2(parameters);
    // odom_pose_preint_ = Pose3();
    std::cout << "Graph object constructed" << std::endl;
}

Graph2D::Graph2D() : GraphND() {}

Graph2D::Graph2D(int &node_cnt, std::vector<float> init_std, std::vector<float> &motion_std,
                 std::vector<float> gps_std, std::vector<float> depth_std) : GraphND(node_cnt)
{
    // 2D prior
    Pose2 prior_pose(0.0, 0.0, 0.0); // prior at origin
    initial_estimate_.insert(X(node_cnt), prior_pose);

    // Store noise models
    motion_std_ = motion_std;
    gps_std_ = gps_std;
    depth_std_ = depth_std;

    // Assemble prior noise model and add it the graph.`
    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(3) << init_std.at(0), init_std.at(1), init_std.at(2))
            .finished()); // rad,rad,rad,m, m, m

    graph_ = new NonlinearFactorGraph();
    graph_->add(PriorFactor<Pose2>(X(node_cnt), prior_pose, pose_noise_model));
}

Graph3D::Graph3D() : GraphND(){}

Graph3D::Graph3D(int &node_cnt, std::vector<float> init_std, std::vector<float> &motion_std,
                 std::vector<float> gps_std, std::vector<float> depth_std) : GraphND(node_cnt)
{
    // 3D prior
    Rot3 prior_rotation = Rot3::Quaternion(1, 0,0,0);
    Point3 prior_point(0,0,0);
    Pose3 prior_pose(prior_rotation, prior_point);
    initial_estimate_.insert(X(node_cnt), prior_pose);
    
    // Store noise models
    motion_std_ = motion_std;
    gps_std_ = gps_std;
    depth_std_ = depth_std;

    //initial_estimate_.insert(R(node_cnt), prior_rotation);

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
        (Vector(6) << init_std.at(0), init_std.at(1), init_std.at(2), init_std.at(3), init_std.at(4), init_std.at(5))
            .finished()); // rad,rad,rad,m, m, m

    // auto pose_noise_model = noiseModel::Diagonal::Sigmas(noise);

    graph_ = new NonlinearFactorGraph();
    // graph_->add(PriorFactor<Pose3>(X(node_cnt), prior_pose, pose_noise_model));

    // Start with a clear 
    result_.clear();
    // graph_->addPrior(V(node_cnt_), prior_velocity, velocity_noise_model);
    // graph_->addPrior(B(node_cnt_), prior_imu_bias, bias_noise_model_);
}

// void Graph2D::OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int &node_cnt, double depth)
void Graph2D::OdomNode(const Vector3 &ang_vel_t, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int node_cnt, double depth)
{
    // // depth_t_ = odom_msg->pose.pose.position.z;

    // Vector3 euler = odom_rotation.rpy();

    // Vector3 odom_step = odom_rotation.matrix() * lin_vel_t * dt;
    // Pose2 odom_pose(odom_pose_prev_.translation()[0] + odom_step[0],
    //                 odom_pose_prev_.translation()[1] + odom_step[1],
    //                 euler[2]);

    // initial_estimate_.insert(X(node_cnt), odom_pose);

    // // Add odometry factors between consecutive poses
    // // Below is equivalent to odom_pose_prev_.between(odom_pose)
    // // Pose2 odom_step = odom_pose_prev_.inverse().compose(odom_pose);
    // // TODO: extract noise from odom_msg
    // noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 1., 1., 0.1).finished());
    // graph_->add(BetweenFactor<Pose2>(X(node_cnt - 1), X(node_cnt), odom_pose_prev_.between(odom_pose), odometryNoise));

    // odom_pose_prev_ = odom_pose;
}

void Graph2D::IntegrateOdom(int &node_cnt)
{

}

void Graph3D::IntegrateOdom(int &node_cnt)
{
    Pose3 prev_odom;
    // If result contains the last previous node
    if (result_.exists(X(node_cnt - 1)))
    {
        // std::cout << "Pre pose is from result " << std::endl;
        prev_odom = result_.at<Pose3>(X(result_.size() - 2));
    }
    // If not, check if it is in the initial estimate.
    // This should be the case until a first optimiziation round has taken place
    else if (initial_estimate_.exists(X(node_cnt - 1)))
    {
        prev_odom = initial_estimate_.at<Pose3>(X(node_cnt - 1));
    }
    // If the last previous node isn't already in results or initial_estimate is because node_cnt-1 is in int_hist.
    // In that case, take the latest initial_estimate node and integrate from there
    else
    {
        // prev_odom = result_.at<Pose3>(X(result_.size() - 2));
        if (!initial_estimate_.empty())
        {
            gtsam::Values::iterator last_it = --initial_estimate_.end();
            // std::cout << "Key from init " << last_it->key << std::endl;
            prev_odom = initial_estimate_.at<Pose3>(last_it->key);
        }
        else
        {
            gtsam::Values::iterator last_it = --result_.end();
            // std::cout << "Key from result " << last_it->key << std::endl;
            prev_odom = result_.at<Pose3>(last_it->key);
        }
    }

    // Copy and reset history immediatly after. Otherwise the int_hist_ might received new info while in the for loop, 
    // leading to that info being lost in the clear()
    std::vector<int_step> int_hist_local = int_hist_;
    int_hist_.clear();

    for(int_step step_i: int_hist_local)
    {
        // In Euler
        Vector3 rot_prev = prev_odom.rotation().rpy();
        Vector3 rot_euler = rot_prev + std::get<2>(step_i) * std::get<3>(step_i);
        // Wrap yaw
        bool was_neg = rot_euler[2] < 0;
        rot_euler[2] = fmod(rot_euler[2], static_cast<double>(2.0 * M_PI));
        if (was_neg)
            rot_euler[2] += static_cast<double>(2.0 * M_PI);
        Rot3 rot_now = Rot3::Ypr(rot_euler[2], rot_euler[1], rot_euler[0]);

        // TODO: do this in quaternions
        // Rot3 rot_prev = odom_pose_prev.rotation();
        // Vector3 euler_step = ang_vel_t * dt;
        // Rot3 rot_step = Rot3::Ypr(euler_step[2], euler_step[1], euler_step[0]);
        // Rot3 rot_now = rot_step * rot_prev;

        Vector3 pos_step = rot_now.matrix() * std::get<1>(step_i) * std::get<3>(step_i);
        Point3 pos_now(prev_odom.translation()[0] + pos_step[0],
                        prev_odom.translation()[1] + pos_step[1],
                        std::get<4>(step_i));
        Pose3 odom_pose(rot_now, pos_now);
        // std::cout << "Odom pose " << odom_pose.translation()[0] << ", " << odom_pose.translation()[1] << ", " << odom_pose.translation()[2] << std::endl;

        // Vector3 odom_velocity(odom_msg->twist.twist.angular.x, odom_msg->twist.twist.angular.y, odom_msg->twist.twist.angular.z);
        // gtsam::NavState odom_estimate(odom_pose, odom_velocity);
        // initial_estimate_.insert(V(node_cnt_), odom_estimate.v());
        // initial_estimate_.insert(B(node_cnt_), prev_bias_);

        // Add odometry factors between consecutive poses
        // Below is equivalent to odom_pose_prev_.between(odom_pose)
        // Pose2 odom_step = odom_pose_prev_.inverse().compose(odom_pose);
        // TODO: extract noise from odom_msg
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6) << motion_std_.at(0), motion_std_.at(1),
                                                                                       motion_std_.at(1), motion_std_.at(2), motion_std_.at(3),
                                                                                       motion_std_.at(5))
                                                                                          .finished());
        BetweenFactor<Pose3> odom_factor(X(std::get<0>(step_i) - 1), X(std::get<0>(step_i)), prev_odom.between(odom_pose), odometryNoise);

        // std::cout << "Odom cnt " << std::get<0>(step_i) << std::endl;

        graph_->add(odom_factor);
        // std::cout << "Adding key to initial_estimate " << std::get<0>(step_i) << std::endl;
        initial_estimate_.insert(X(std::get<0>(step_i)), odom_pose);

        // Add prior on roll and pitch
        // Vector3 r = odom_pose.rotation().xyz();
        // gtsam::Unit3 nG = gtsam::Rot3::RzRyRx(r.x(), r.y(), 0).rotate(gtsam::Unit3(0, 0, -1));
        gtsam::Unit3 nG = gtsam::Unit3(0, 0, -1);
        gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector2(0.1, 10));
        graph_->add(Pose3GravityFactor(X(std::get<0>(step_i) - 1), gtsam::Unit3(0, 0, -1), model, Unit3(0, 0, 1)));

        // graph_->add(PriorFactor<Pose3GravityFactor>(X(std::get<0>(step_i)), Pose3GravityFactor(X(std::get<0>(step_i)), nG, model, Unit3(0, 0, 1)), model));
        // graph.add(gtsam::PriorFactor<gtsam::Pose2>(rootId, gtsam::Pose2(initialPose.x(), initialPose.y(), initialPose.theta()), priorNoise));

        prev_odom = odom_pose;
    }
}

// void Graph3D::OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int &node_cnt, double depth)
void Graph3D::OdomNode(const Vector3 &ang_vel_t, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int node_cnt, double depth)
{
    int_hist_.push_back(int_step(node_cnt, lin_vel_t, ang_vel_t, dt, depth));

    // If we can catch the lock: take the latest odom and integrate the int_hist
    if (graph_mux_.try_lock())
    {
        this->IntegrateOdom(node_cnt);
        graph_mux_.unlock();
    }
}


void Graph3D::DepthPrior(int cnt, double depth)
{
    // Depth constraint for 3D case
    noiseModel::Diagonal::shared_ptr depthNoise = noiseModel::Diagonal::Sigmas((Vector(1) << depth_std_.at(0)).finished());
    Pose3DepthFactor depth_factor(X(cnt), depth, depthNoise);

    if (!graph_mux_.try_lock())
    {
        depth_factors_.push_back(depth_factor);
    }
    else
    {
        if(!depth_factors_.empty())
        {
            for (auto depth_fact: depth_factors_)
            {
                graph_->add(depth_fact);
            }
            depth_factors_.clear();
        }
        else
        {
            graph_->add(depth_factor);
            // graph_->add(boost::make_shared<Pose3DepthFactor>(X(cnt), depth, depthNoise));
        }
        graph_mux_.unlock();
    }


    // graph_->add(depth_factor);
}

void Graph2D::GpsNode(const std::vector<double> &gps_odom, int node_cnt, double depth)
{
    // 2D version
    auto unaryNoise = noiseModel::Isotropic::Sigma(2, gps_std_.at(0));
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

void Graph3D::GpsNode(const std::vector<double> &gps_odom, int node_cnt, double depth)
{
    // 3D version
    auto correction_noise = noiseModel::Isotropic::Sigmas((Vector(3) << 25., 25., 0.1).finished());
    GPSFactor gps_factor(X(node_cnt),
                         Point3(gps_odom.at(0),
                                gps_odom.at(1),
                                depth),
                         correction_noise);

    if (!graph_mux_.try_lock())
    {
        std::cout << "Saving GPS meas for later " << std::endl;
        gps_factors_.push_back(gps_factor);
    }
    else
    {
        std::cout << "Adding GPS meas" << std::endl;
        if (!gps_factors_.empty())
        {
            for (auto gps_fact : gps_factors_)
            {
                graph_->add(gps_fact);
            }
            gps_factors_.clear();
        }
        else
        {
            graph_->add(gps_factor);
            // graph_->add(boost::make_shared<Pose3DepthFactor>(X(cnt), depth, depthNoise));
        }
        graph_mux_.unlock();
    }
}

void Graph2D::Optimize(int cnt)
{
    // // Optimize

    // // iSAM2
    // std::cout << "----------------- Optimizing --------------------" << std::endl;
    // // writeG2o(*graph_, initial_estimate_, "before.dot");
    // ISAM2Result update_info = isam2_->update(*graph_, initial_estimate_);
    // update_info.print();

    // // Update odom estimate
    // result_ = isam2_->calculateEstimate();
    // // odom_pose_prev_ = result_.at<Pose2>(X(cnt));

    // // Reset the graph
    // graph_->resize(0);
    // initial_estimate_.clear();

    // // Reset the preintegration object.
    // // preintegrated_->resetIntegrationAndSetBias(prev_bias_);
}

void Graph3D::Optimize(int cnt)
{

    // iSAM2
    // If optimizing, integrate odom in the meantime
    try
    {
        std::cout << "Odom node cnt " << cnt << std::endl;
        std::cout << "Results size " << result_.size() << std::endl;
        std::cout << "Initial estimate size " << initial_estimate_.size() << std::endl;
        int it = 0;
        // odom_pose_preint_ = initial_estimate_.at<Pose3>(X(node_cnt));
        while(!graph_mux_.try_lock() && it < 10 )
        {
            // Dangerous shit
            sleep(0.05);
            it++;
            std::cout << "Optimizer waiting for the lock" << std::endl;
        }
        if(it < 10)
        {

            std::cout << "----------------- Optimizing --------------------" << std::endl;

            this->IntegrateOdom(cnt);
            std::cout << "DR preintegrated" << std::endl;
            std::cout << "Odom node cnt " << cnt << std::endl;
            std::cout << "Results size " << result_.size() << std::endl;
            std::cout << "Initial estimate size " << initial_estimate_.size() << std::endl;

            // if(graph_mux_.try_lock())
            // {
            // writeG2o(*graph_, initial_estimate_, "before.dot");
            ISAM2Result update_info = isam2_->update(*graph_, initial_estimate_);
            isam2_->update();
            isam2_->update();
            isam2_->update();
            update_info.print();

            // result_ = isam2_->calculateEstimate(X(cnt));
            result_ = isam2_->calculateEstimate();
            std::cout << " Estimate calculated " << std::endl;
            
            graph_->resize(0);
            initial_estimate_.clear();
            std::cout << " Graph and estimate reset " << std::endl;


            std::cout << "----------------- Optimization done --------------------" << std::endl;
        }
        else
        {
            std::cout << "----------------- Optimization: Missed lock --------------------" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "Graph loc node. Optimize step: " << e.what() << std::endl;
        std::cout << "=========================================================================================" << std::endl;
    }
    graph_mux_.unlock();

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
    Quaternion rot = pose_i.rotation().toQuaternion();

    return std::vector<double>{pose_i.translation()[0], pose_i.translation()[1], pose_i.translation()[2],
                               rot.x(), rot.y(), rot.z(), rot.w()};
}

void Graph3D::SBGPrior(const Rot3& sbg_rotation, int cnt)
{
    double headingSigma = 0.1;
    auto noiseModel = noiseModel::Constrained::MixedSigmas(Vector3(0.0, 0.0, headingSigma));
    graph_->emplace_shared<gtsam::PriorFactor<gtsam::Rot3>>(
        R(cnt), sbg_rotation, noiseModel);

    // Add current rot estimate from DR as initial value
    gtsam::Rot3 r_estimate;
    if(result_.exists(R(cnt)))
    {
        r_estimate = result_.at<gtsam::Rot3>(R(cnt));
    }
    else if (initial_estimate_.exists(R(cnt)))
    {
        r_estimate = initial_estimate_.at<gtsam::Rot3>(R(cnt));
    }

    initial_estimate_.insert(R(cnt), r_estimate);
    // graph_->add(PriorFactor<Pose3>(X(node_cnt), prior_pose, pose_noise_model));
}

// bool Graph3D::CopyGraph(Graph3D graph_copy)
// {
//     if(graph_mux_.try_lock())
//     {
//         graph_copy = new Graph3D (*this);
//     //     graph_mux_.unlock();
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// Graph3D* Graph3D::clone() {
//     return new Graph3D(*graph_);
// }

// bool Graph2D::CopyGraph(Graph2D graph_copy)
// {
//     // if (graph_mux_.try_lock())
//     // {
//     //     graph_copy = boost::make_shared<Graph2D>(*graph_);
//     //     return true;
//     // }
//     // else
//     // {
//         return false;
//     // }
// }
