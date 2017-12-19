#include "lolo_auv_nav/lolo_auv_nav.hpp"
#include "mbes_receptor/mbes_receptor.hpp"

#include <ros/callback_queue.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "lolo_auv_nav_node");

    ros::NodeHandle nh_nav;
    ros::NodeHandle nh_sss;
    ros::CallbackQueue sss_queue;
    ros::CallbackQueue nav_queue;
    nh_nav.setCallbackQueue(&nav_queue);
    nh_sss.setCallbackQueue(&sss_queue);

    LoLoEKF *lolo_auv_nav = new LoLoEKF(ros::this_node::getName(), nh_nav);
    MBESReceptor* sss_scanner = new MBESReceptor(ros::this_node::getName(), nh_sss);

    ros::AsyncSpinner spinner_nav(1, &nav_queue);
    ros::AsyncSpinner spinner_sss(1, &sss_queue);
    spinner_nav.start();
    spinner_sss.start();

    ros::waitForShutdown();

    if(!ros::ok()){
        delete lolo_auv_nav;
        delete sss_scanner;
    }

    return 0;
}
