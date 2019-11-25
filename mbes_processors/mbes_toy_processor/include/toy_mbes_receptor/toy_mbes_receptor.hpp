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

#ifndef TOY_MBES_RECEPTOR_CPP
#define TOY_MBES_RECEPTOR_CPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "toy_mbes_receptor/toy_mbes_manipulator.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MyTimerPolicy;

/**
 * @brief The MBESReceptor class
 * This class is intended to be an interface between two sonar sensors providing landmark positions
 * It just synchronizes the reception of inputs and transforms them to the base_link frame
 */

class MBESReceptor{

public:
    MBESReceptor(std::string node_name, ros::NodeHandle &nh_right, ros::NodeHandle &nh_left);
    ~MBESReceptor();

private:
    std::string node_name_;
    ros::NodeHandle* nh_right_;
    ros::NodeHandle* nh_left_;
    ros::NodeHandle nh_;

    std::string sss_r_frame_;
    std::string sss_l_frame_;
    std::string base_frame_;
    tf::StampedTransform tf_base_sss_r_;
    tf::StampedTransform tf_base_sss_l_;

    message_filters::Subscriber<sensor_msgs::LaserScan>* mbes_l_subs_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* mbes_r_subs_;
    message_filters::Synchronizer<MyTimerPolicy>* mbes_synch_;
    ros::Publisher pcl_pub_;
    ros::Publisher landmark_pub_;

    // Create processors for sonar inputs
    SonarManipulator sss_right_;
    SonarManipulator sss_left_;

    /**
     * @brief mbesReadingsCB
     * @param mbes_l_msg
     * @param mbes_r_msg
     * Synchs inputs, transforms them and outputs landmarks positions as PointStamped
     */
    void mbesReadingsCB(const sensor_msgs::LaserScanConstPtr & mbes_l_msg, const sensor_msgs::LaserScanConstPtr &mbes_r_msg );

    /**
     * @brief init
     * Instantiation of rigid transforms
     */
    void init();
};

#endif // TOY_MBES_RECEPTOR_CPP
