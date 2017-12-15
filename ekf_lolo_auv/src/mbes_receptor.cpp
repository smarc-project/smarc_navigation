#include "mbes_receptor/mbes_receptor.hpp"


MBESReceptor::MBESReceptor(std::string node_name, ros::NodeHandle &nh): node_name_(node_name), nh_(&nh){

    std::string mbes_left_tp;
    std::string mbes_right_tp;
    std::string pcl_out_top;

    nh_->param<std::string>((node_name_ + "/mbes_left_topic"), mbes_left_tp, "/sss_left");
    nh_->param<std::string>((node_name_ + "/mbes_right_topic"), mbes_right_tp, "/sss_right");
    nh_->param<std::string>((node_name_ + "/pcl_out"), pcl_out_top, "/pcl_out");

    // Synch reception of mbes msgs
    mbes_l_subs_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh_, mbes_left_tp, 20);
    mbes_r_subs_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh_, mbes_right_tp, 20);
    mbes_synch_ = new message_filters::Synchronizer<MyTimerPolicy>(MyTimerPolicy(50), *mbes_l_subs_, *mbes_r_subs_);
    mbes_synch_->registerCallback(boost::bind(&MBESReceptor::mbesReadingsCB, this, _1, _2));

    // RVIZ pcl output for testing
    pcl_pub_ = nh_->advertise<pcl::PointCloud<pcl::PointXYZ>>(pcl_out_top, 10);

}


void MBESReceptor::mbesReadingsCB(const sensor_msgs::LaserScanConstPtr &mbes_l_msg,
                                  const sensor_msgs::LaserScanConstPtr &mbes_r_msg){

    pcl::PointCloud<pcl::PointXYZ> cloud_in;

    // Fill in the CloudIn data
    cloud_in.width    = 5;
    cloud_in.height   = 1;
    cloud_in.is_dense = false;
    cloud_in.points.resize (cloud_in.width * cloud_in.height);

    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    {
      cloud_in.points[i].x = 10 * rand ();
      cloud_in.points[i].y = 10 * rand ();
      cloud_in.points[i].z = 10 * rand ();
    }

    tf::TransformListener sss_base_listener;

    try{
        sss_base_listener.waitForTransform("/lolo_auv/base_link", "/lolo_auv/sonarleft_link", mbes_l_msg->header.stamp, ros::Duration(10));
        cloud_in.header.stamp = mbes_l_msg->header.stamp.toSec();
        cloud_in.header.frame_id = "lolo_auv/base_link";
        pcl_pub_.publish(cloud_in);
        ROS_INFO("PointCloud published");
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
        ros::Duration(1.0).sleep();
    }

}


MBESReceptor::~MBESReceptor(){

}
