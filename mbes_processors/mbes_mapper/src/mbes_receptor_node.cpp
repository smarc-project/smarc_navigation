#include "mbes_mapper/mbes_receptor.hpp"


int main(int argc, char** argv){

    ros::init(argc, argv, "mbes_receptor_node");

    ros::NodeHandle nh;
    MBESReceptor* mbes_receptor = new MBESReceptor(ros::this_node::getName(), nh);

    ros::waitForShutdown();

    if(!ros::ok()){
        delete mbes_receptor;
    }
    ROS_INFO("Finishing MBES receptor");

    return 0;
}
