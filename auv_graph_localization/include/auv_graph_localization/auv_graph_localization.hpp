
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/navigation/MagFactor.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>
#include <future>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

// #pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam
{

    class Pose3DepthFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
    {

    private:
        // measurement information
        double mz_;

    public:
        Pose3DepthFactor(gtsam::Key poseKey, const double &m, gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), mz_(m) {}

        gtsam::Vector evaluateError(const gtsam::Pose3 &X, boost::optional<gtsam::Matrix &> J1 = boost::none) const
        {

            if (J1)
                *J1 = (gtsam::Matrix16() << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();

            return (gtsam::Vector1() << X.z() - mz_).finished();
        }
    };

    class Pose3PitchFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
    {

    private:
        // measurement information
        double mpitch_;

    public:
        Pose3PitchFactor(gtsam::Key poseKey, const double &m, gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), mpitch_(m) {}

        gtsam::Vector evaluateError(const gtsam::Pose3 &X, boost::optional<gtsam::Matrix &> J1 = boost::none) const
        {

            if (J1)
                *J1 = (gtsam::Matrix16() << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0).finished();

            return (gtsam::Vector1() << X.rotation().rpy()[1] - mpitch_).finished();
        }
    };

    class Pose3RollFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
    {

    private:
        // measurement information
        double mroll_;

    public:
        Pose3RollFactor(gtsam::Key poseKey, const double &m, gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), mroll_(m) {}

        gtsam::Vector evaluateError(const gtsam::Pose3 &X, boost::optional<gtsam::Matrix &> J1 = boost::none) const
        {

            if (J1)
                *J1 = (gtsam::Matrix16() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished();

            return (gtsam::Vector1() << X.rotation().rpy()[0] - mroll_).finished();
        }
    };
}

class UnaryFactor: public NoiseModelFactor1<Pose2> {
  double mx_, my_; ///< X and Y measurements

public:
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

    Vector evaluateError(const Pose2& q,
                        boost::optional<Matrix&> H = boost::none) const
    {
        const Rot2& R = q.rotation();
        if (H) (*H) = (gtsam::Matrix(2, 3) <<
                R.c(), -R.s(), 0.0,
                R.s(), R.c(), 0.0).finished();
        return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
    }
};

class GraphLocalization
{

public:
    ros::NodeHandle *nh_;
    ros::NodeHandle *nh_stim_;
    ros::Subscriber odom_sub_, stim_sub_, gps_sub_;
    ros::Publisher path_pub_, preint_pub_;
    std::string odom_frame_, map_frame_, utm_frame_;
    NonlinearFactorGraph *graph_;
    ISAM2 *isam2_;
    Values initial_estimate_;
    Values result_;
    std::vector<Values> path_;
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p_;
    std::shared_ptr<PreintegrationType> preintegrated_;
    int node_cnt_;
    int stim_cnt_;

    bool stim_init_, odom_init_;
    bool optimized_;
    double stim_t_now_, odom_t_now_;
    double stim_t_prev_, odom_t_prev_;
    float vis_rate_;
    double depth_t_;

    // NavState *prev_state_;
    NavState prop_state_;
    imuBias::ConstantBias prev_bias_;
    SharedIsotropic bias_noise_model_;
    Pose3 odom_pose_prev_;
    tf2_ros::Buffer tf_buffer_;
    geometry_msgs::TransformStamped utm_odom_tf_;

    GraphLocalization(ros::NodeHandle &nh, ros::NodeHandle &nh_stim);

    void StimCb(const sensor_msgs::ImuConstPtr& imu_msg);

    void OdomCb(const nav_msgs::OdometryConstPtr &odom_msg);

    void GpsCb(const nav_msgs::OdometryConstPtr &gps_msg);

    void Visualize();

    void Optimize(int cnt);

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> stimParams();
};