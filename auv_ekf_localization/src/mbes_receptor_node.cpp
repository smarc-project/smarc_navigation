#include "mbes_receptor/mbes_receptor.hpp"


int main(int argc, char** argv){

    ros::init(argc, argv, "lolo_mbes_node");

    ros::NodeHandle nh_sss_r;
    ros::NodeHandle nh_sss_l;
    ros::CallbackQueue sss_r_queue;
    ros::CallbackQueue sss_l_queue;
    nh_sss_r.setCallbackQueue(&sss_r_queue);
    nh_sss_l.setCallbackQueue(&sss_l_queue);

    MBESReceptor* sss_scanner = new MBESReceptor(ros::this_node::getName(), nh_sss_r, nh_sss_l);

    ros::AsyncSpinner spinner_sss_right(1, &sss_r_queue);
    ros::AsyncSpinner spinner_sss_left(1, &sss_l_queue);
    spinner_sss_right.start();
    spinner_sss_left.start();

    ros::waitForShutdown();

    if(!ros::ok()){
        delete sss_scanner;
    }

    return 0;
}
