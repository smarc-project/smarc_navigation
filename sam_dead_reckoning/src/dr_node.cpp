#include "sam_dead_recknoning/dr_provider.hpp"

#include <ros/callback_queue.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "sam_dr_node");

    ros::NodeHandle nh;
    boost::shared_ptr<drNode> sam_dr_node(new drNode(ros::this_node::getName(), nh));

    ros::spin();
    ros::waitForShutdown();

    if(!ros::ok()){
        sam_dr_node.reset();
    }

    return 0;
}