//PID Transform Listener Node for SAM, Sriharsha Bhat,  7 Dec 2018
#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
//#include <tf2_msgs/TFMessage.h>
//#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <smarc_msgs/DVL.h>

std_msgs::Float64 current_alt;

//Callback to read altitude from DVL
void DVLCallback(const smarc_msgs::DVL& dvl_msg)
{
    current_alt.data = dvl_msg.altitude;
    ROS_INFO_THROTTLE(1.0, "[ tf_listener]  Altitude from DVL: %f", dvl_msg.altitude);
}

int main(int argc, char** argv){

  std::string node_name = "tf_listener";
  ros::init(argc, argv, node_name);

  ros::NodeHandle node;

  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;
  std::string topic_from_dvl_;
  double freq;

  ros::NodeHandle node_priv("~");
  node_priv.param<std::string>("base_frame", base_frame_, "/sam/base_link");
  node_priv.param<std::string>("world_frame", map_frame_, "/world_ned");
  //node.param<std::string>("odom_frame", odom_frame_, "/odom");
  node_priv.param<std::string>("topic_from_dvl", topic_from_dvl_, "/sam/core/dvl");
  node_priv.param<double>("loop_freq", freq, 10);

  //initiate subscribers
  ros::Subscriber dvl_sub = node.subscribe(topic_from_dvl_, 1, DVLCallback);

//initiate publishers
  ros::Publisher feedback_pitch = node.advertise<std_msgs::Float64>("pitch", freq);
  ros::Publisher feedback_roll = node.advertise<std_msgs::Float64>("roll", freq);
  ros::Publisher feedback_yaw = node.advertise<std_msgs::Float64>("yaw", freq);
  ros::Publisher feedback_depth = node.advertise<std_msgs::Float64>("depth", freq);
  ros::Publisher feedback_x = node.advertise<std_msgs::Float64>("x", freq);
  ros::Publisher feedback_y = node.advertise<std_msgs::Float64>("y", freq);
  ros::Publisher feedback_u = node.advertise<std_msgs::Float64>("u", freq);
  ros::Publisher feedback_v = node.advertise<std_msgs::Float64>("v", freq);
  ros::Publisher feedback_w = node.advertise<std_msgs::Float64>("w", freq);
  ros::Publisher feedback_p = node.advertise<std_msgs::Float64>("p", freq);
  ros::Publisher feedback_q = node.advertise<std_msgs::Float64>("q", freq);
  ros::Publisher feedback_r = node.advertise<std_msgs::Float64>("r", freq);
  ros::Publisher feedback_alt = node.advertise<std_msgs::Float64>("altitude", freq);

//Variable initialization
  tf::TransformListener listener;
  std_msgs::Float64 current_roll,current_pitch,current_yaw,current_depth, current_x,current_y, current_u, current_v, current_w, current_p,current_q, current_r;
  double r,p,y;
  tf::Quaternion tfq;

  //Define the transforms to make this modular : can be done once you have a working system
  //node_priv.param<std::string>("topic_from_controller", topic_from_controller_, "control_effort");


  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    geometry_msgs::Twist  twist;
    try{//transform between utm and required frame
      listener.lookupTransform(map_frame_, base_frame_,
                               ros::Time(0), transform);
      listener.lookupTwist(map_frame_, base_frame_,
                               ros::Time(0), ros::Duration(2.0), twist);
    }

    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    //get orientation with quaternions
    tfq = transform.getRotation();
    tf::Matrix3x3(tfq).getEulerYPR(y,p,r);

    //orientation
    current_pitch.data= p;
    current_roll.data= r;
    current_yaw.data= y;
    current_depth.data= -transform.getOrigin().z();
    current_x.data= transform.getOrigin().x();
    current_y.data= transform.getOrigin().y();

    //Velocity
    current_u.data= twist.linear.x;
    current_v.data= twist.linear.y;
    current_w.data= twist.linear.z;
    current_p.data= twist.angular.x;
    current_q.data= twist.angular.y;
    current_r.data= twist.angular.z;

    //Publish orientation
    feedback_pitch.publish(current_pitch);
    feedback_roll.publish(current_roll);
    feedback_yaw.publish(current_yaw);
    feedback_depth.publish(current_depth);
    feedback_x.publish(current_x);
    feedback_y.publish(current_y);

    //Publish velocity
    feedback_u.publish(current_u);
    feedback_v.publish(current_v);
    feedback_w.publish(current_w);
    feedback_p.publish(current_p);
    feedback_q.publish(current_q);
    feedback_r.publish(current_r);

    //Publish DVL altitude
    feedback_alt.publish(current_alt);


    ROS_INFO_THROTTLE(1.0, "[ tf_listener ] roll: %f, pitch: %f, yaw: %f, depth: %f, x: %f, y: %f, u: %f, v: %f, w: %f, p: %f, q: %f, r: %f alt:%f", current_roll.data,current_pitch.data,current_yaw.data,current_depth.data, current_x.data, current_y.data, current_u.data, current_v.data, current_w.data, current_p.data, current_q.data, current_r.data, current_alt.data);

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
};
