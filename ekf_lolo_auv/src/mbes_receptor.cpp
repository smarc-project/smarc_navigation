#include "mbes_receptor/mbes_receptor.hpp"


MBESReceptor::MBESReceptor(std::string node_name, ros::NodeHandle &nh_right, ros::NodeHandle &nh_left):
    node_name_(node_name), nh_right_(&nh_right), nh_left_(&nh_left){

    std::string mbes_left_tp;
    std::string mbes_right_tp;
    std::string lm_detect_top;

    nh_left_->param<std::string>((node_name_ + "/mbes_left_topic"), mbes_left_tp, "/sss_left");
    nh_right_->param<std::string>((node_name_ + "/mbes_right_topic"), mbes_right_tp, "/sss_right");
    nh_left_->param<std::string>((node_name_ + "/sss_l_link"), sss_l_frame_, "/sss_link");
    nh_right_->param<std::string>((node_name_ + "/sss_r_link"), sss_r_frame_, "/sss_link");

    nh_.param<std::string>((node_name_ + "/lm_detect_topic"), lm_detect_top, "/landmarks_detected");
    nh_.param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");

    // Synch reception of mbes msgs
    mbes_l_subs_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh_left_, mbes_left_tp, 20);
    mbes_r_subs_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh_right_, mbes_right_tp, 20);
    mbes_synch_ = new message_filters::Synchronizer<MyTimerPolicy>(MyTimerPolicy(50), *mbes_l_subs_, *mbes_r_subs_);
    mbes_synch_->registerCallback(boost::bind(&MBESReceptor::mbesReadingsCB, this, _1, _2));

    // RVIZ pcl output for testing
    landmark_pub_ = nh_.advertise<geometry_msgs::PointStamped>(lm_detect_top, 10);

    this->init();

}

void MBESReceptor::init(){
    tf::TransformListener tf_listener;
    try{
        tf_listener.waitForTransform(base_frame_, sss_r_frame_, ros::Time(0), ros::Duration(100));
        tf_listener.lookupTransform(base_frame_, sss_r_frame_, ros::Time(0), tf_sss_r_base_);
        ROS_INFO("Locked transform sss right --> base");
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
        ros::Duration(1.0).sleep();
    }

    try{
        tf_listener.waitForTransform(base_frame_, sss_l_frame_, ros::Time(0), ros::Duration(100));
        tf_listener.lookupTransform(base_frame_, sss_l_frame_, ros::Time(0), tf_sss_l_base_);
        ROS_INFO("Locked transform sss left --> base");
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
        ros::Duration(1.0).sleep();
    }
}

auto print = [](const double& n) { std::cout << " " << n; };



void MBESReceptor::mbesReadingsCB(const sensor_msgs::LaserScanConstPtr &mbes_l_msg,
                                  const sensor_msgs::LaserScanConstPtr &mbes_r_msg){

    // Process right input
    sss_right_.processSonarInput(mbes_r_msg);

    // If any higher intensity value detected
    if(sss_right_.landmarks_.size() > 0){
        // Transform to base frame
        tf::Vector3 aux_vec2 = tf_sss_r_base_ * sss_right_.landmarks_.back();

        // Create marker for RVIZ
//        tf::Stamped<tf::Vector3> lm_base(tf_sss_r_base_ * aux_vec, ros::Time(0), "lolo_auv/base_link");
//        geometry_msgs::Vector3Stamped landmark_msg;   // Move to PointCloud and expand for more than one landmark detected
//        tf::vector3StampedTFToMsg(lm_base, landmark_msg);
        geometry_msgs::PointStamped landmark_msg;
        landmark_msg.header.stamp = mbes_r_msg->header.stamp;
        landmark_msg.header.frame_id = "lolo_auv/base_link";
        landmark_msg.point.x = aux_vec2.x();
        landmark_msg.point.y = aux_vec2.y();
        landmark_msg.point.z = aux_vec2.z();

        landmark_pub_.publish(landmark_msg);
    }
    sss_right_.landmarks_.clear();
}


MBESReceptor::~MBESReceptor(){

}

















