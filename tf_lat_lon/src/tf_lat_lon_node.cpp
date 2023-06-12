#include <ros/ros.h>
#include <geodesy/utm.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <smarc_msgs/LatLonStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <sensor_msgs/NavSatFix.h>

#include <smarc_msgs/UTMToLatLon.h>
#include <smarc_msgs/LatLonToUTM.h>
#include <smarc_msgs/NEDENURotation.h>
#include <smarc_msgs/LatLonOdometry.h>
#include <smarc_msgs/LatLonToUTMOdometry.h>

using namespace std;

//void fromMsg(const geographic_msgs::GeoPoint &from, UTMPoint &to,
//        const bool& force_zone, const char& band, const uint8_t& zone)

class TFUTMConversion {
protected:

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2_ros::StaticTransformBroadcaster staticBroadcaster;
    //tf2_ros::TransformBroadcaster tfBroadcaster;

    std::string frame;
    int utm_zone; // 33
    std::string utm_band; // N

public:

    geometry_msgs::Point convert_point(const geographic_msgs::GeoPoint& lat_lon_point)
    {
        geodesy::UTMPoint geo_utm_point;
	geo_utm_point.zone = utm_zone;
	geo_utm_point.band = utm_band[0];
	//geodesy::fromMsg(lat_lon_point, geo_utm_point, true, utm_band[0], utm_zone);
        geodesy::fromMsg(lat_lon_point, geo_utm_point);
        // ENU -> x=easting, y=northing, z=altitude
        geometry_msgs::Point utm_point;
        utm_point.x = geo_utm_point.easting;
        utm_point.y = geo_utm_point.northing;
        utm_point.z = geo_utm_point.altitude;
        return utm_point;
    }

    geometry_msgs::Quaternion convert_orientation(const geometry_msgs::Quaternion& req)
    {
        geometry_msgs::Quaternion res;

        tf2::Quaternion req_quat;
        tf2::convert(req, req_quat);
        tf2::Matrix3x3 req_mat(req_quat);

        tf2::Matrix3x3 ned_to_enu_mat(0., 1., 0., 1., 0., 0., 0., 0., -1.);
        tf2::Matrix3x3 vehicle_switch_z_mat(1., 0., 0., 0., -1., 0., 0., 0., -1.);
        tf2::Matrix3x3 transformed_mat = ned_to_enu_mat*req_mat*vehicle_switch_z_mat;

        tf2::Quaternion transformed_quat;
        transformed_mat.getRotation(transformed_quat);
        tf2::convert(transformed_quat, res);

        /*
        double roll, pitch, yaw;
        req_mat.getRPY(roll, pitch, yaw);
        std::cout << "Original Roll: " << 180./M_PI*roll << " pitch: " << 180./M_PI*pitch << " yaw: " << 180./M_PI*yaw << std::endl;
        transformed_mat.getRPY(roll, pitch, yaw);
        std::cout << "Transformed Roll: " << 180./M_PI*roll << " pitch: " << 180./M_PI*pitch << " yaw: " << 180./M_PI*yaw << std::endl;
        */

        return res;
    }

    nav_msgs::Odometry convert_lat_lon_odom(const smarc_msgs::LatLonOdometry& lat_lon_odom)
    {
        nav_msgs::Odometry odom;
        odom.pose.pose.orientation = convert_orientation(lat_lon_odom.lat_lon_pose.orientation);
        odom.pose.pose.position = convert_point(lat_lon_odom.lat_lon_pose.position);

        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("utm", "map", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_INFO("Could not get map transform, publishing first one...");
            transformStamped.transform.translation.x = odom.pose.pose.position.x;
            transformStamped.transform.translation.y = odom.pose.pose.position.y;
            transformStamped.transform.translation.z = 0.;
            transformStamped.transform.rotation.w = 1.;
            transformStamped.header.frame_id = "utm";
            transformStamped.child_frame_id = "map";
            transformStamped.header.stamp = ros::Time::now();
            staticBroadcaster.sendTransform(transformStamped);
        }
        
        odom.pose.pose.position.x -= transformStamped.transform.translation.x;
        odom.pose.pose.position.y -= transformStamped.transform.translation.y;
        odom.pose.pose.position.z -= transformStamped.transform.translation.z;

        return odom;
    }

    TFUTMConversion(const std::string& frame, int utm_zone, const std::string& utm_band)
        : tfListener(tfBuffer), frame(frame), utm_zone(utm_zone), utm_band(utm_band)
    {

    }

};

class TFLatLonServer : public TFUTMConversion {
private:

    ros::Publisher lat_lon_pub;
    ros::Publisher vis_gps_pub;
    ros::Timer lat_lon_timer;
    bool published_vis_gps;
    
    ros::ServiceServer lat_lon_to_utm_odom_service;
    ros::ServiceServer lat_lon_to_utm_service;
    ros::ServiceServer utm_to_lat_lon_service;
    ros::ServiceServer enu_to_ned_rot_service;
    ros::ServiceServer ned_to_enu_rot_service;

public:

    bool translate_odometry(smarc_msgs::LatLonToUTMOdometry::Request& req,
                            smarc_msgs::LatLonToUTMOdometry::Response& res)
    {
        res.odom = convert_lat_lon_odom(req.lat_lon_odom);
        if (!published_vis_gps) {
            sensor_msgs::NavSatFix gps;
            gps.header.stamp = ros::Time::now();
            gps.header.frame_id = "map";
            gps.latitude = req.lat_lon_odom.lat_lon_pose.position.latitude;
            gps.longitude = req.lat_lon_odom.lat_lon_pose.position.longitude;
            gps.altitude = 0.;
            gps.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            vis_gps_pub.publish(gps);
            published_vis_gps = true;
        }
        return true;
    }

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
        res.utm_point = convert_point(req.lat_lon_point);
        return true;
    }

    bool translate_rotation(smarc_msgs::NEDENURotation::Request& req,
                            smarc_msgs::NEDENURotation::Response& res)
    {
        res.orientation = convert_orientation(req.orientation);
        return true;
    }

    TFLatLonServer(ros::NodeHandle& nh) : TFUTMConversion("", 0, ""), published_vis_gps(false)
    {
        ros::NodeHandle pn("~");
        pn.param<std::string>("frame", frame, "sam/base_link");
        pn.param<int>("utm_zone", utm_zone, 33);
        pn.param<string>("utm_band", utm_band, "V");
        
        ROS_INFO("Starting tf lat lon server with UTM zone: %d, band: %s...", utm_zone, utm_band.c_str());

        //tfListener = tf2_ros::TransformListener(tfBuffer);
        lat_lon_pub = nh.advertise<geographic_msgs::GeoPoint>("tf_lat_lon", 1000);
        vis_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("vis_gps", 10, true); // latching publisher
        lat_lon_timer = nh.createTimer(ros::Duration(0.1), &TFLatLonServer::timer_callback, this);

        lat_lon_to_utm_odom_service = nh.advertiseService("lat_lon_to_utm_odom", &TFLatLonServer::translate_odometry, this);
        lat_lon_to_utm_service = nh.advertiseService("lat_lon_to_utm", &TFLatLonServer::translate_lat_lon, this);
        utm_to_lat_lon_service = nh.advertiseService("utm_to_lat_lon", &TFLatLonServer::translate_utm, this);
        enu_to_ned_rot_service = nh.advertiseService("enu_to_ned_rot", &TFLatLonServer::translate_rotation, this);
        ned_to_enu_rot_service = nh.advertiseService("ned_to_enu_rot", &TFLatLonServer::translate_rotation, this);
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
