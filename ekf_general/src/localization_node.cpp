#include "localization.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;

    Localization* ext_kalman_filter = new Localization(ros::this_node::getName(), nh);

    if(!ros::ok()){
        delete ext_kalman_filter;
    }

    return 0;
}
