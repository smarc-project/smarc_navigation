
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

using namespace std;
using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

int main(int argc, char *argv[])
{
    // Rot3 odom_rotation = Rot3::Quaternion(0.8775826, 0, 0, 0.4794255);
    // Point3 odom_point(10,0,0);
    // Pose3 odom_pose(odom_rotation, odom_point);

    // Rot3 odom_rotation_prev = Rot3::Quaternion(1, 0, 0, 0);
    // Point3 odom_point_prev(0, -10, 0);
    // Pose3 odom_pose_prev(odom_rotation_prev, odom_point_prev);

    // Pose3 odom_step = odom_pose.compose(odom_pose_prev.inverse());
    // cout << "Odom step: " << odom_step.translation()[0] << ", " << odom_step.translation()[1] << ", " << odom_step.translation()[2] << "\t "
    //         << "\n";

    // // Rot3 odom_rotation = Rot3::Quaternion(1.,
    // //                                       0.,
    // //                                       0.,
    // //                                       0.);
    // Vector3 euler = odom_rotation.rpy();
    // cout << "Euler: " << euler[0] << ", " << euler[1] << ", " << euler[2] << "\t "
    //      << "\n";

    // Pose2 now(17.5727, -8.32464, -1.46435);
    // Pose2 prev(17.4934, -7.51033, -1.38913);
    // Pose2 step(0.693182, 0.479094, -0.0752221);

    // // These two are different somehow
    // Pose2 odom = prev.inverse().compose(now);
    // odom.print();
    // odom = prev.between(now);
    // odom.print();

    // // Magnetometer
    // Point3 nM(22653.29982, -1956.83010, 44202.47862);
    // // Let's assume scale factor,
    // double scale = 255.0 / 50000.0;
    // // ...ground truth orientation,
    // Rot3 nRb = Rot3::Yaw(-0.1);
    // Rot2 theta = nRb.yaw();
    // // ...and bias
    // Point3 bias(10, -10, 50);
    // // ... then we measure
    // Point3 scaled = scale * nM;
    // Point3 measured = nRb.inverse() * (scale * nM) + bias;

    // std::cout << scaled << std::endl;

    // Point3 expected(22735.5, 314.502, 44202.5);
    // Matrix H;

    // std::cout << "Norm " <<  nM.norm() << std::endl;
    // double s(scale * nM.norm());
    // SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
    // Unit3 dir(nM);

    // gtsam::MagFactor f(1, measured, s, dir, bias, model);

    // ros::init(argc, argv, "graph_localization");

    // ROS
    // ros::NodeHandle nh("~");
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    ISAM2 *isam2_ = new ISAM2(parameters);
    NonlinearFactorGraph *graph_;
    Values initial_estimate_;
    NonlinearFactorGraph::shared_ptr graph;
    Values::shared_ptr initial;
    Values result_;

    boost::tie(graph, initial) = readG2o("/home/torroba/.ros/before.dot", true);
    ISAM2Result update_info = isam2_->update(*graph, *initial);
    // isam2_->update();
    // isam2_->update();
    // isam2_->update();
    update_info.print();

    // result_ = isam2_->calculateEstimate(X(cnt));
    result_ = isam2_->calculateEstimate();
    std::cout << " Estimate calculated " << std::endl;
}