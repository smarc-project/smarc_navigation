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

#ifndef MBES_RECEPTOR_HPP
#define MBES_RECEPTOR_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <tuple>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <fstream>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class MBESReceptor{

public:
    MBESReceptor(std::string node_name, ros::NodeHandle &nh);
    ~MBESReceptor();

private:
    std::string node_name_;
    ros::NodeHandle* nh_;

    ros::Publisher pcl_pub_;
    ros::Publisher landmark_pub_;
    ros::Subscriber mbes_laser_sub_;

    std::string mbes_frame_;
    std::string base_frame_;
    std::string map_frame_;

    tf::StampedTransform tf_base_mbes_;
    tf::StampedTransform tf_base_map_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster submaps_bc_;

    PointCloud::Ptr pcl_msg_;
    laser_geometry::LaserProjection projector_;

    // Aux
    unsigned int pcl_cnt_;
    int meas_size_;

    // Submaps
    std::vector<std::tuple<PointCloud, tf::Transform>> mbes_swath_;
    std::vector<tf::Transform> tf_map_meas_vec_;

    void MBESLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan_in);

    void bcMapSubmapsTF(std::vector<tf::Transform> tfs_meas_map);

    void savePointCloud(PointCloud submap_pcl, std::string file_name);

    void pclFuser();

    void init();
};

#endif // MBES_RECEPTOR_HPP
