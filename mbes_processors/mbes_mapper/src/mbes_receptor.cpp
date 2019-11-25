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

#include "mbes_mapper/mbes_receptor.hpp"


MBESReceptor::MBESReceptor(std::string node_name, ros::NodeHandle &nh):
    node_name_(node_name), nh_(&nh){

    std::string mbes_topic;
    std::string pcl_pub_topic;

    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/mbes_frame"), mbes_frame_, "/mbes_link");
    nh_->param<std::string>((node_name_ + "/map_frame"), map_frame_, "/map_link");
    nh_->param<int>((node_name_ + "/submap_size"), meas_size_, 5);
    nh_->param<std::string>((node_name_ + "/mbes_laser_topic"), mbes_topic, "/mbes_laser_topic");
    nh_->param<std::string>((node_name_ + "/pcl_pub_topic"), pcl_pub_topic, "/pcl_pub_topic");

    // RVIZ pcl output for testing
    mbes_laser_sub_ = nh_->subscribe(mbes_topic, 10, &MBESReceptor::MBESLaserCB, this);
    pcl_pub_ = nh_->advertise<sensor_msgs::PointCloud2> (pcl_pub_topic, 2);

    this->init();
    ros::spin();
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

Eigen::Matrix4f MBESReceptor::inverseTfMatrix(Eigen::Matrix4f tf_mat){

    Eigen::Matrix3f R_inv = tf_mat.topLeftCorner(3,3).transpose();

    Eigen::Matrix4f tf_mat_inv = Eigen::Matrix4f::Identity();
    tf_mat_inv.topLeftCorner(3,3) = R_inv;
    tf_mat_inv.topRightCorner(3,1) = R_inv * (-tf_mat.topRightCorner(3,1));

    return tf_mat_inv;

}


void MBESReceptor::pclFuser(){

    ROS_INFO("PCL fuser called");

    // Check time stamps for max distance covered in the new swath

    // Build and store frame of new submap meas
    tf::Transform tf_submap_map = std::get<1>(mbes_swath_.at((meas_size_-1)/2));
    tf_map_meas_vec_.push_back(tf_submap_map.inverse());

    // Broadcast all meas frames (for testing?)
    bcMapSubmapsTF(tf_map_meas_vec_);

    // Transform and concatenate the PCLs to form the swath
    PointCloud tfed_pcl;
    PointCloud submap_pcl;

    // For each ping
    for(std::tuple<PointCloud, tf::Transform> ping: mbes_swath_){
        // Transform from base at t to meas frame
        pcl_ros::transformPointCloud(std::get<0>(ping), tfed_pcl, tf_submap_map * std::get<1>(ping).inverse());
        // Add to submap pcl
        submap_pcl += tfed_pcl;
    }

    // Store tf pose in submap
    Eigen::Vector3d tf_vec;
    tf::vectorTFToEigen (tf_submap_map.inverse().getOrigin(), tf_vec);
    submap_pcl.sensor_origin_ << tf_vec.cast<float>(), 0;

    Eigen::Quaterniond tf_quat;
    tf::quaternionTFToEigen (tf_submap_map.inverse().getRotation(), tf_quat);
    submap_pcl.sensor_orientation_ = tf_quat.cast<float>();

    // Create ROS msg and publish
    sensor_msgs::PointCloud2 submap_msg;
    pcl::toROSMsg(submap_pcl, submap_msg);
    submap_msg.header.frame_id = "submap_" + std::to_string(tf_map_meas_vec_.size()-1) + "_frame";
    submap_msg.header.stamp = ros::Time::now();
    pcl_pub_.publish(submap_msg);

    // Save pcls in .pdc
    pcl::io::savePCDFileASCII("./submap_" + std::to_string(tf_map_meas_vec_.size()-1) + "_frame.pdc", submap_pcl);
}

void MBESReceptor::bcMapSubmapsTF(std::vector<tf::Transform> tfs_meas_map){

    int cnt_i = 0;
    tf::StampedTransform tf_map_submap_stp;
    geometry_msgs::TransformStamped msg_map_submap;
    for(tf::Transform tf_measi_map: tfs_meas_map){
         tf_map_submap_stp = tf::StampedTransform(tf_measi_map,
                                                  ros::Time::now(),
                                                  map_frame_,
                                                  "submap_" + std::to_string(cnt_i) + "_frame");

         cnt_i += 1;
         tf::transformStampedTFToMsg(tf_map_submap_stp, msg_map_submap);
         submaps_bc_.sendTransform(msg_map_submap);
    }
}

void MBESReceptor::MBESLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan_in){

    // LaserScan to PCL
    if(!tf_listener_.waitForTransform(mbes_frame_,
                                      base_frame_,
                                      scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                                      ros::Duration(0.1))){
        ROS_INFO("Skipping iteration");
        // TODO: handle this somehow?
        return;
    }

    sensor_msgs::PointCloud2 scan_cloud;
    PointCloud pcl_cloud;
    projector_.transformLaserScanToPointCloud(base_frame_, *scan_in, scan_cloud, tf_listener_);

    // Convert from PointCloud2 to PCL pointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(scan_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

    // Listen to tf map --> base pose
    try {
        tf_listener_.waitForTransform(base_frame_, map_frame_, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(0.1));
        tf_listener_.lookupTransform(base_frame_, map_frame_, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), tf_base_map_);
        ROS_DEBUG_STREAM(node_name_ << ": locked transform map --> base at t");

        // Store both PCL and tf at meas time
        mbes_swath_.emplace_back(pcl_cloud, tf::Transform(tf_base_map_.getRotation().normalize(), tf_base_map_.getOrigin()));
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
    }

    // When desired num of pings per swath is reached, merge them and clear buffer
    if(mbes_swath_.size() == meas_size_){
        this->pclFuser();
        mbes_swath_.clear();
    }
}

MBESReceptor::~MBESReceptor(){

}




