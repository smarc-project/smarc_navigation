/* Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "toy_mbes_receptor/toy_mbes_receptor.hpp"


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
    landmark_pub_ = nh_.advertise<geometry_msgs::PoseArray>(lm_detect_top, 10);

    this->init();

}

void MBESReceptor::init(){
    tf::TransformListener tf_listener;
    try{
        tf_listener.waitForTransform(base_frame_, sss_r_frame_, ros::Time(0), ros::Duration(100));
        tf_listener.lookupTransform(base_frame_, sss_r_frame_, ros::Time(0), tf_base_sss_r_);
        ROS_INFO("Locked transform sss right --> base");
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
        ros::Duration(1.0).sleep();
    }

    try{
        tf_listener.waitForTransform(base_frame_, sss_l_frame_, ros::Time(0), ros::Duration(100));
        tf_listener.lookupTransform(base_frame_, sss_l_frame_, ros::Time(0), tf_base_sss_l_);
        ROS_INFO("Locked transform sss left --> base");
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
        ros::Duration(1.0).sleep();
    }
}

void MBESReceptor::mbesReadingsCB(const sensor_msgs::LaserScanConstPtr &mbes_l_msg,
                                  const sensor_msgs::LaserScanConstPtr &mbes_r_msg){

    // Process MBES inputs from both sides
    sss_right_.processSonarInput(mbes_r_msg);
    sss_left_.processSonarInput(mbes_l_msg);

    geometry_msgs::PoseArray lm_detected;
    lm_detected.header.stamp = mbes_r_msg->header.stamp;
    lm_detected.header.frame_id = base_frame_;
    geometry_msgs::Pose landmark_pose;
    tf::Vector3 lm_pose;

    // If any higher intensity value detected
    if(!sss_right_.landmarks_.empty()){
        for(auto landmark: sss_right_.landmarks_){
            // Transform sss_right_sensor --> base_link
            lm_pose = tf_base_sss_r_ * landmark;
            landmark_pose.position.x = lm_pose.x();
            landmark_pose.position.y = lm_pose.y();
            landmark_pose.position.z = lm_pose.z();
            lm_detected.poses.push_back(landmark_pose);
        }
    }
    if(!sss_left_.landmarks_.empty()){
        for(auto landmark: sss_left_.landmarks_){
            // Transform sss_right_sensor --> base_link
            lm_pose = tf_base_sss_l_ * landmark;
            landmark_pose.position.x = lm_pose.x();
            landmark_pose.position.y = lm_pose.y();
            landmark_pose.position.z = lm_pose.z();
            lm_detected.poses.push_back(landmark_pose);
        }
    }

    // If landmarks detected, publish and clean up
    if(!lm_detected.poses.empty()){
        landmark_pub_.publish(lm_detected);
    }
    sss_right_.landmarks_.clear();
    sss_left_.landmarks_.clear();
}


MBESReceptor::~MBESReceptor(){

}

















