
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

using namespace std;
using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

int main()
{
    Rot3 odom_rotation = Rot3::Quaternion(1,0,0,0);
    Point3 odom_point(10,0,0);
    Pose3 odom_pose(odom_rotation, odom_point);

    Rot3 odom_rotation_prev = Rot3::Quaternion(1, 0, 0, 0);
    Point3 odom_point_prev(0, -10, 0);
    Pose3 odom_pose_prev(odom_rotation_prev, odom_point_prev);

    Pose3 odom_step = odom_pose.compose(odom_pose_prev.inverse());
    cout << "Odom step: " << odom_step.translation()[0] << ", " << odom_step.translation()[1] << ", " << odom_step.translation()[2] << "\t "
            << "\n";
}