#include "ekf_slam/ekf_slam.hpp"

#include <ros/callback_queue.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "auv_nav_node");

    ros::NodeHandle nh_nav;
    ros::CallbackQueue nav_queue;
    nh_nav.setCallbackQueue(&nav_queue);

    boost::shared_ptr<EKFSLAM> auv_nav(new EKFSLAM(ros::this_node::getName(), nh_nav));

    ros::AsyncSpinner spinner_nav(1, &nav_queue);
    spinner_nav.start();

    ros::waitForShutdown();

    if(!ros::ok()){
        auv_nav.reset();
    }

    return 0;
}
