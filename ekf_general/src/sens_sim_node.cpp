#include "sens_sim.hpp"


int main(int argc, char ** argv){
    ros::init(argc, argv, "sens_sim_node");
    ros::NodeHandle nh;
    SensSim *sens_sim_obj = new SensSim(ros::this_node::getName(), nh);

    if(!ros::ok()){
        delete sens_sim_obj;
    }

    return 0;
}
