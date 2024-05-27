#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>
#include <future>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <ros/callback_queue.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <auv_graph_localization/graph.hpp>

using namespace std;
using namespace gtsam;

class GraphLocalization
{

public:
    ros::NodeHandle *nh_;
    ros::NodeHandle *nh_stim_;
    ros::NodeHandle *nh_gps_;
    ros::Subscriber odom_sub_, stim_sub_, gps_sub_, aux_sub_, uwgps_odom_sub_;
    ros::Publisher path_pub_, preint_pub_, loc_pub_;
    std::string base_frame_, odom_frame_, map_frame_, utm_frame_;
    boost::shared_ptr<GraphND> graph_;

    int node_cnt_;
    int stim_cnt_;

    bool stim_init_, odom_init_;
    bool optimized_;
    double stim_t_now_, odom_t_now_;
    double stim_t_prev_, odom_t_prev_;
    float vis_rate_;
    double depth_t_;
    bool aux_bool_;

    nav_msgs::Odometry odom_msg_;

    tf2_ros::Buffer tf_buffer_;
    geometry_msgs::TransformStamped utm_odom_tf_;

    GraphLocalization(ros::NodeHandle &nh, ros::NodeHandle &nh_stim, ros::NodeHandle &nh_gps);

    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    geometry_msgs::TransformStamped tf_odom_base_;

    void StimCb(const sensor_msgs::ImuConstPtr& imu_msg);

    void UWGPSOdomCb(const nav_msgs::OdometryConstPtr &uwgps_odom);

    void OdomCb(const nav_msgs::OdometryConstPtr &odom_msg);

    void GpsCb(const nav_msgs::OdometryConstPtr &gps_msg);
    
    void AuxCb(const std_msgs::BoolConstPtr &aux_msg);

    void Visualize();

    void Optimize(int cnt);

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> stimParams();
};