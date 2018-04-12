#include "mbes_mapper/mbes_receptor.hpp"


MBESReceptor::MBESReceptor(std::string node_name, ros::NodeHandle &nh):
    node_name_(node_name), nh_(&nh){

    std::string mbes_topic;
    std::string pcl_pub_topic;

    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/mbes_frame"), mbes_frame_, "/mbes_link");
    nh_->param<std::string>((node_name_ + "/mbes_laser_topic"), mbes_topic, "/mbes_laser_topic");
    nh_->param<std::string>((node_name_ + "/pcl_pub_topic"), pcl_pub_topic, "/pcl_pub_topic");


    // Synch reception of mbes msgs
    // RVIZ pcl output for testing
    mbes_laser_sub_ = nh_->subscribe(mbes_topic, 10, &MBESReceptor::MBESLaserCB, this);
    pcl_pub_ = nh_->advertise<sensor_msgs::PointCloud> (pcl_pub_topic, 2);

    this->init();

}

void MBESReceptor::init(){
    tf::TransformListener tf_listener;
    try{
        tf_listener.waitForTransform(base_frame_, mbes_frame_, ros::Time(0), ros::Duration(100));
        tf_listener.lookupTransform(base_frame_, mbes_frame_, ros::Time(0), tf_base_mbes_);
        ROS_INFO_STREAM(node_name_ << ": Locked transform MBES --> base");
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
    }
    ROS_INFO_STREAM(node_name_ << ": launched");

}


void MBESReceptor::MBESLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan_in){

    if(!tf_listener_.waitForTransform(
        mbes_frame_,
        base_frame_,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(2.0))){
        ROS_INFO("Skipping iteration");
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud(base_frame_, *scan_in, cloud, tf_listener_);

//  PointCloud::Ptr msg (new PointCloud);
//  msg->header.frame_id = base_frame_;
//  msg->points.push_back(cloud);

  ROS_INFO("Publishing PCL");
  pcl_pub_.publish(cloud);

}

MBESReceptor::~MBESReceptor(){

}




