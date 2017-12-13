#include "mbes_receptor/mbes_receptor.hpp"



MBESReceptor::MBESReceptor(std::string node_name, ros::NodeHandle &nh): node_name_(node_name), nh_(&nh){

    std::string mbes_left_tp;
    std::string mbes_right_tp;

    nh_->param<std::string>((node_name_ + "/mbes_left_topic"), mbes_left_tp, "/sss_left");
    nh_->param<std::string>((node_name_ + "/mbes_right_topic"), mbes_right_tp, "/sss_right");

    // Synch reception of mbes msgs
    mbes_l_subs_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh_, mbes_left_tp, 10);
    mbes_r_subs_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh_, mbes_right_tp, 10);
    mbes_synch_ = new message_filters::Synchronizer<MyTimerPolicy>(MyTimerPolicy(10), *mbes_l_subs_, *mbes_r_subs_);
    mbes_synch_->registerCallback(boost::bind(&MBESReceptor::mbesReadingsCB, this, _1, _2));

}


void MBESReceptor::mbesReadingsCB(const sensor_msgs::LaserScanConstPtr &mbes_l_msg,
                                  const sensor_msgs::LaserScanConstPtr &mbes_r_msg){


}


MBESReceptor::~MBESReceptor(){

}
