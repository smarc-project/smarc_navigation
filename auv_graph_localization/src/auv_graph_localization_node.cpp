#include "auv_graph_localization/auv_graph_localization.hpp"

#include <ros/ros.h>

using namespace std;
using namespace gtsam;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "graph_localization");

    // ROS
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_stim("~");
    ros::NodeHandle nh_gps("~");
    ros::CallbackQueue stim_queue;
    ros::CallbackQueue general_queue;
    ros::CallbackQueue gps_queue;
    nh_stim.setCallbackQueue(&stim_queue);
    nh.setCallbackQueue(&general_queue);
    nh_gps.setCallbackQueue(&general_queue);

    boost::shared_ptr<GraphLocalization> graph_loc(new GraphLocalization(nh, nh_stim, nh_gps));

    // Spinner for STIM
    ros::AsyncSpinner spinner_stim(2, &stim_queue);
    spinner_stim.start();

    // Spinner for Odom
    ros::AsyncSpinner spinner_odom(2, &general_queue);
    spinner_odom.start();

    // Spinner for GPS
    ros::AsyncSpinner spinner_gps(2, &gps_queue);
    spinner_gps.start();

    ros::waitForShutdown();
    if (!ros::ok())
    {
        graph_loc.reset();
    }

    return 0;
}