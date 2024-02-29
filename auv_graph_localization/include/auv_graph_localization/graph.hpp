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
#include <gtsam/navigation/MagFactor.h>

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
            {
                Vector3 euler = X.rotation().rpy();
                *J1 = (gtsam::Matrix16() << 0.0, 0.0, 0.0, -sin(euler[1]), cos(euler[1]) * sin(euler[0]), cos(euler[1]) * cos(euler[0])).finished();

                return (gtsam::Vector1() << X.z() - mz_).finished();
            }
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
}

class UnaryFactor : public NoiseModelFactor1<Pose2>
{
    double mx_, my_; ///< X and Y measurements

public:
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel &model) : NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

    Vector evaluateError(const Pose2 &q,
                         boost::optional<Matrix &> H = boost::none) const
    {
        const Rot2 &R = q.rotation();
        if (H)
            (*H) = (gtsam::Matrix(2, 3) << 
                    R.c(), -R.s(), 0.0,
                    R.s(), R.c(), 0.0).finished();
        return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
    }
};

class GraphND
{
public:
    NonlinearFactorGraph *graph_;
    ISAM2 *isam2_;
    Values initial_estimate_;
    Values result_;
    std::vector<Values> path_;
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p_;
    std::shared_ptr<PreintegrationType> preintegrated_;
    // NavState *prev_state_;
    NavState prop_state_;
    imuBias::ConstantBias prev_bias_;
    SharedIsotropic bias_noise_model_;

    // T odom_pose_prev_;

    GraphND(int &node_cnt);

    virtual void OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, double dt, int &node_cnt, double depth) {}

    virtual void GpsNode(const std::vector<double> &gps_odom, int &node_cnt){}

    virtual void Optimize(int cnt){}

    virtual std::vector<double> getValue(Values &values, int i){}

    virtual void DepthPrior(int cnt, double depth){}
};

class Graph2D: public GraphND
{
public:

    Pose2 odom_pose_prev_;

    Graph2D(int &node_cnt);

    void OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, double dt, int &node_cnt, double depth);

    void GpsNode(const std::vector<double> &gps_odom, int &node_cnt);

    void Optimize(int cnt);

    std::vector<double> getValue(Values &values, int i);
};

class Graph3D: public GraphND
{
public:

    Pose3 odom_pose_prev_;

    Graph3D(int &node_cnt);

    void OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, double dt, int &node_cnt, double depth);

    void GpsNode(const std::vector<double> &gps_odom, int &node_cnt);

    void Optimize(int cnt);

    void DepthPrior(int cnt, double depth);

    std::vector<double> getValue(Values &values, int i);
};