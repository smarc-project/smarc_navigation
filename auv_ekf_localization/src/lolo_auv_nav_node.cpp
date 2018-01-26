#include "lolo_auv_nav/lolo_auv_nav.hpp"

#include <ros/callback_queue.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "lolo_auv_nav_node");

    ros::NodeHandle nh_nav;
    ros::CallbackQueue nav_queue;
    nh_nav.setCallbackQueue(&nav_queue);

    LoLoEKF *lolo_auv_nav = new LoLoEKF(ros::this_node::getName(), nh_nav);

    ros::AsyncSpinner spinner_nav(1, &nav_queue);
    spinner_nav.start();

    ros::waitForShutdown();

    if(!ros::ok()){
        delete lolo_auv_nav;
    }

    return 0;
}
