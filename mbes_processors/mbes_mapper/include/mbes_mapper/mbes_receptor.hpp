#ifndef MBES_RECEPTOR_HPP
#define MBES_RECEPTOR_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

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

    tf::StampedTransform tf_base_mbes_;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener tf_listener_;

    void MBESLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan_in);

    void init();
};

#endif // MBES_RECEPTOR_HPP
