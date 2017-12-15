#include "mbes_receptor/mbes_receptor.hpp"

int main(int argc, char** argv){

    ros::init(argc, argv, "mbes_receptor_node");
    ros::NodeHandle nh;

    MBESReceptor* sss_scanner = new MBESReceptor(ros::this_node::getName(), nh);

    ros::spin();
    if(!ros::ok()){
        delete sss_scanner;
    }

    return 0;
}
