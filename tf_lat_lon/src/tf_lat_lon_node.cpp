#include <ros/ros.h>
#include <geodesy/utm.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <smarc_msgs/LatLonStamped.h>
#include <geographic_msgs/GeoPoint.h>

using namespace std;

tf2_ros::Buffer tfBuffer;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_lat_lon_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    std::string frame;
    int utm_zone; // 33
    std::string utm_band; // N

    pn.param<std::string>("frame", frame, "sam/base_link");
    pn.param<int>("/utm_zone", utm_zone, 33);
    pn.param<string>("/utm_band", utm_band, "V");

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Publisher pub = nh.advertise<geographic_msgs::GeoPoint>("output", 1000);

    ros::Rate r(10); // 10 hz
    while (ros::ok()) {

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.transform.translation.x = 0.;
        transformStamped.transform.translation.y = 0.;
        try {
            transformStamped = tfBuffer.lookupTransform("world", frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            //continue;
        }

        double easting = transformStamped.transform.translation.x;
        double northing = transformStamped.transform.translation.y;

        std::cout << "utm_zone: " << utm_zone << ", utm_band: " << utm_band[0] << std::endl;
        
        geodesy::UTMPoint utm_point(easting, northing, utm_zone, utm_band[0]);
        geographic_msgs::GeoPoint msg = geodesy::toMsg(utm_point);
        pub.publish(msg);

        r.sleep();
    }

    return 0;
}
