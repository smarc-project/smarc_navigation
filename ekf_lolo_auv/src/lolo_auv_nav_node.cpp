#include "lolo_auv_nav/lolo_auv_nav.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "lolo_auv_nav_node");
    ros::NodeHandle nh;
    LoLoEKF *lolo_auv_nav = new LoLoEKF(ros::this_node::getName(), nh);

    lolo_auv_nav->ekfLocalize();

    if(!ros::ok()){
        delete lolo_auv_nav;
    }

    return 0;
}
