#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
//#include <rfs_slam/rfs_pkgConfig.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

// #include "contour_blob_detector/contour_blob_detector.h"

using namespace std;

class RockDetectionServer {

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher shape_pub;
    ros::Publisher detections_pub;
    tf::StampedTransform tf_base_fls_;
    string base_frame_;
    string fls_frame_;

//    dynamic_reconfigure::Server<rfs_slam::rfs_pkgConfig> server_;
//    dynamic_reconfigure::Server<rfs_slam::rfs_pkgConfig>::CallbackType f_;
    boost::mutex shared_mutex_;

    double grid_size;
    double max_radius;
    int blur_x_;
    int blur_y_;
    float min_area_;
    float max_area_;
    float min_circ_;

public:

    void callback(const sensor_msgs::Image::ConstPtr& image_msg)
    {

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(-1);
        }

        cv::Mat image8, smoothed;
        cv_ptr->image.convertTo(image8, CV_8UC1, 255.0);
        cv::GaussianBlur(image8, smoothed, cv::Size(blur_x_ ,blur_y_), blur_x_);

        // Setup SimpleBlobDetector parameters.
        cv::SimpleBlobDetector::Params params;

        // Change thresholds
        params.minDistBetweenBlobs = 100.0f;

        params.filterByInertia = false;
        params.filterByConvexity = false;
        params.filterByColor = false;

        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = min_circ_;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = min_area_;
        params.maxArea = max_area_;

        // Set up the detector with default parameters.
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

        // Detect blobs.
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(smoothed, keypoints);

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        cv::Mat im_with_keypoints;
        drawKeypoints(smoothed, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        geometry_msgs::PoseArray detections_msg;
        detections_msg.header.frame_id = fls_frame_;
        detections_msg.header.stamp = ros::Time::now();
        for(const cv::KeyPoint& k : keypoints){

            // Transform keypoints to fls frame
            tf::Vector3 lm_fls = tf::Vector3(image8.rows - k.pt.y - 1,
                                             k.pt.x - image8.cols/2,
                                             0);

            geometry_msgs::Pose pose_msg;
            pose_msg.position.x = lm_fls.getX();
            pose_msg.position.y = lm_fls.getY();
            pose_msg.position.z = lm_fls.getZ();

            pose_msg.orientation.x = 0.0f;
            pose_msg.orientation.y = 0.0f;
            pose_msg.orientation.z = 1.0f;
            pose_msg.orientation.w = 0.0f;

            detections_msg.poses.push_back(pose_msg);
        }
        // Publish only if sth has been detected
        if(!detections_msg.poses.empty()){
            detections_pub.publish(detections_msg);
        }

        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->encoding = "rgb8";
        cv_pub_ptr->image = im_with_keypoints;
        sensor_msgs::Image ros_image = *cv_pub_ptr->toImageMsg();
        pub.publish(ros_image);

//        ROS_INFO("Published image");
    }

//    void dynParamsCB(rfs_slam::rfs_pkgConfig &config, uint32_t)
//    {
//        blur_x_ = config.blur_size_x;
//        blur_y_ = config.blur_size_y;
//        min_circ_ = config.min_circ;
//        min_area_ = config.min_area;
//        max_area_ = config.max_area;
//    }

    RockDetectionServer(const std::string& name)
    {

        ros::NodeHandle pn("~");
        pn.param<double>("grid_size", grid_size, 0.2);
        pn.param<double>("max_radius", max_radius, 20.0);

        string detections_topic;
        string sonar_img_topic;

        // Init dynamic params
        blur_x_ = 3;
        blur_y_ = 13;
        min_area_ = 600.0;
        max_area_ = 5000.0;
        min_circ_ = 0.1;

//        f_ = boost::bind(&RockDetectionServer::dynParamsCB, this, _1, _2);
//        server_.setCallback(f_);

        n.param<std::string>((name + "/base_frame"), base_frame_, "/base_frame");
        n.param<std::string>((name + "/fls_frame"), fls_frame_, "/fls_frame");
        n.param<std::string>((name + "/lm_detect_topic"), detections_topic, "/velodyne_detections");
        n.param<std::string>((name + "/image_raw_sonar"), sonar_img_topic, "/depth/image_raw_raw_sonar");

        detections_pub = n.advertise<geometry_msgs::PoseArray>(detections_topic, 1);
        sub = n.subscribe(sonar_img_topic, 1, &RockDetectionServer::callback, this);
        pub = n.advertise<sensor_msgs::Image>("/velodyne_blobs", 1);
//        shape_pub = n.advertise<visualization_msgs::MarkerArray>("/velodyne_outlines", 1);

        tf::TransformListener tf_listener;
        try{
            tf_listener.waitForTransform(base_frame_, fls_frame_, ros::Time(0), ros::Duration(100));
            tf_listener.lookupTransform(base_frame_, fls_frame_, ros::Time(0), tf_base_fls_);
            ROS_INFO_STREAM(name <<  ": Locked transform fls --> base");
        }
        catch(tf::TransformException &exception){
            ROS_ERROR("%s", exception.what());
            ros::Duration(1.0).sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fls_rock_detector");

    RockDetectionServer rds(ros::this_node::getName());

    ros::spin();

    return 0;
}
