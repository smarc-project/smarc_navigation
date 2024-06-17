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

using symbol_shorthand::R;        // Rotation
using symbol_shorthand::B;        // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;        // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;        // Pose3 (x,y,z,r,p,y)
// #pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AttitudeFactor.h>
// #include <gtsam/navigation/GravityFactor.h>
#include <auv_graph_localization/gravity_factor.hpp>

#include <boost/thread.hpp>
#include <chrono>
#include <thread>

namespace gtsam
{
    class Pose3DepthFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
    {

    private:
        // measurement information
        double mz_;

    public:
        Pose3DepthFactor(gtsam::Key poseKey, const double &m, gtsam::SharedNoiseModel model) : 
                        gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), mz_(m) {}

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

  

        // class Pose3HeadingFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
        // {

        // private:
        //     // measurement information
        //     double myaw_;

        // public:
        //     Pose3HeadingFactor(gtsam::Key poseKey, const double &m, gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), myaw_(m) {}

        //     gtsam::Vector evaluateError(const gtsam::Pose3 &X, boost::optional<gtsam::Matrix &> J1 = boost::none) const
        //     {

        //         if (J1)
        //         {
        //             X.matrix() * Pose3::Expmap()
        //             Vector3 euler = X.rotation().rpy();
        //             *J1 = (gtsam::Matrix16() << 0.0, 0.0, 0.0, -sin(euler[1]), cos(euler[1]) * sin(euler[0]), cos(euler[1]) * cos(euler[0])).finished();

        //             // TODO: compute error with quaternions
        //             return (gtsam::Vector1() << X.rotation().yaw() - myaw_).finished();
        //         }
        //     }
        // };

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
                    (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0,
                            R.s(), R.c(), 0.0)
                               .finished();
                return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
            }
        };

    }

    class GraphND
    {

    public:
        NonlinearFactorGraph *graph_;
        ISAM2 *isam2_;
        Values initial_estimate_;
        Values result_;
        // std::vector<Values> path_;
        boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p_;
        std::shared_ptr<PreintegrationType> preintegrated_;
        Pose3 odom_pose_preint_;
        // NavState *prev_state_;
        NavState prop_state_;
        imuBias::ConstantBias prev_bias_;
        SharedIsotropic bias_noise_model_;
        std::mutex graph_mux_;
        std::vector<Pose3DepthFactor> depth_factors_;
        std::vector<GPSFactor> gps_factors_;
        std::vector<BetweenFactor<Pose3>> odom_factors_;
        Values temp_estimate_;

        typedef std::tuple<int, Vector3, Vector3, double, double> int_step;
        std::vector<int_step> int_hist_;

        // T odom_pose_prev_;

        GraphND(int &node_cnt);

        GraphND();

        // virtual void OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int &node_cnt, double depth) {}
        virtual void OdomNode(const Vector3 &ang_vel_t, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int node_cnt, double depth) {}

        virtual void GpsNode(const std::vector<double> &gps_odom, int node_cnt, double depth) {}

        virtual void Optimize(int cnt) {}

        virtual std::vector<double> getValue(Values &values, int i) {}

        virtual void DepthPrior(int cnt, double depth) {}

        virtual void IntegrateOdom(Pose3 &prev_odom, const std::vector<int_step> &int_hist) {}

        // virtual bool CopyGraph(GraphND graph_copy) {}
        // virtual GraphND* Clone() {}
    };

    class Graph2D : public GraphND
    {
    public:
        Pose2 odom_pose_prev_;

        Graph2D(int &node_cnt);

        Graph2D();

        // void OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int &node_cnt, double depth);
        void OdomNode(const Vector3 &ang_vel_t, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int node_cnt, double depth);

        void GpsNode(const std::vector<double> &gps_odom, int node_cnt, double depth);

        void Optimize(int cnt);

        std::vector<double> getValue(Values &values, int i);

        void IntegrateOdom(int &node_cnt);

        // bool CopyGraph(Graph2D graph_copy);

        // void CopyGraph();
    };

    class Graph3D : public GraphND
    {
    public:
        Pose3 odom_pose_prev_;

        Graph3D(int &node_cnt);

        Graph3D();

        // void OdomNode(const Rot3 &odom_rotation, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int &node_cnt, double depth);
        void OdomNode(const Vector3 &ang_vel_t, const Vector3 &lin_vel_t, Pose3 odom_pose_prev, double dt, int node_cnt, double depth);

        void GpsNode(const std::vector<double> &gps_odom, int node_cnt, double depth);

        void Optimize(int cnt);

        void DepthPrior(int cnt, double depth);

        std::vector<double> getValue(Values &values, int i);

        void SBGPrior(const Rot3 &sbg_rotation, int cnt);

        void IntegrateOdom(int &node_cnt);

        // boost::shared_ptr<Graph3D> CopyGraph();
        // bool CopyGraph(Graph3D graph_copy);
    };