#include "odom_provider/odom_provider.hpp"

#include <ros/callback_queue.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "odom_provider_node");

    ros::NodeHandle nh;
    boost::shared_ptr<OdomProvider> odom_provider(new OdomProvider(ros::this_node::getName(), nh));

    ros::spin();
    ros::waitForShutdown();

    if(!ros::ok()){
        odom_provider.reset();
    }

    return 0;
}
