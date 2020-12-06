#include <ros/ros.h>
#include <geodesy/utm.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <smarc_msgs/LatLonStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <smarc_msgs/UTMToLatLon.h>
#include <smarc_msgs/LatLonToUTM.h>

using namespace std;

//void fromMsg(const geographic_msgs::GeoPoint &from, UTMPoint &to,
//        const bool& force_zone, const char& band, const uint8_t& zone)

class TFLatLonServer {
private:

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Publisher lat_lon_pub;
    ros::Timer lat_lon_timer;

    std::string frame;
    int utm_zone; // 33
    std::string utm_band; // N

public:

    bool translate_utm(smarc_msgs::UTMToLatLon::Request& req,
                       smarc_msgs::UTMToLatLon::Response& res)
    {
        // ENU -> x=easting, y=northing, z=altitude
        geodesy::UTMPoint utm_point(req.utm_point.x, req.utm_point.y, req.utm_point.z, utm_zone, utm_band[0]);
        res.lat_lon_point = geodesy::toMsg(utm_point);
        return true;
    }

    bool translate_lat_lon(smarc_msgs::LatLonToUTM::Request& req,
                           smarc_msgs::LatLonToUTM::Response& res)
    {
        geodesy::UTMPoint utm_point;
        geodesy::fromMsg(req.lat_lon_point, utm_point, true, utm_band[0], utm_zone);
        // ENU -> x=easting, y=northing, z=altitude
        res.utm_point.x = utm_point.easting;
        res.utm_point.y = utm_point.northing;
        res.utm_point.z = utm_point.altitude;
        return true;
    }

    TFLatLonServer(ros::NodeHandle& nh) : tfListener(tfBuffer)
    {
        ros::NodeHandle pn("~");
        pn.param<std::string>("frame", frame, "sam/base_link");
        pn.param<int>("/utm_zone", utm_zone, 32);
        pn.param<string>("/utm_band", utm_band, "V");
        
        ROS_INFO("Starting tf lat lon server with UTM zone: %d, band: %s...", utm_zone, utm_band);

        //tfListener = tf2_ros::TransformListener(tfBuffer);
        lat_lon_pub = nh.advertise<geographic_msgs::GeoPoint>("output", 1000);
        lat_lon_timer = nh.createTimer(ros::Duration(0.1), &TFLatLonServer::timer_callback, this);

        ros::ServiceServer lat_lon_to_utm_service = nh.advertiseService("lat_lon_to_utm", &TFLatLonServer::translate_lat_lon, this);
        ros::ServiceServer utm_to_lat_lon_service = nh.advertiseService("utm_to_lat_lon", &TFLatLonServer::translate_utm, this);
    }

    void timer_callback(const ros::TimerEvent& e)
    {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("utm", frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Could not get lat/lon: %s", ex.what());
            return;
        }

        double easting = transformStamped.transform.translation.x;
        double northing = transformStamped.transform.translation.y;
        double altitude = transformStamped.transform.translation.z;

        geodesy::UTMPoint utm_point(easting, northing, altitude, utm_zone, utm_band[0]);
        geographic_msgs::GeoPoint msg = geodesy::toMsg(utm_point);
        lat_lon_pub.publish(msg);
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_lat_lon_node");
    ros::NodeHandle nh;

    TFLatLonServer server(nh);
    ros::spin();

    return 0;
}
