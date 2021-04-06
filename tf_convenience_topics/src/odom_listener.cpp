//Listener Node to the Odometry topic to read positions and velocities in the local frame for SAM, Sriharsha Bhat,  29.5.2020
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <smarc_msgs/DVL.h>

//Variable initialization
std_msgs::Float64 current_roll,current_pitch,current_yaw,current_depth, current_x,current_y, current_u, current_v, current_w, current_p,current_q, current_r, current_alt;
double r,p,y;
tf::Quaternion tfq;

//Callback to read altitude from DVL
void DVLCallback(const smarc_msgs::DVL& dvl_msg)
{
    current_alt.data = dvl_msg.altitude;
   // ROS_INFO_THROTTLE(1.0, "[ odom_listener]  Altitude from DVL: %f", dvl_msg.altitude);
}

//Callback to read states from odomety
void OdomCallback(const nav_msgs::Odometry& odom_msg)
{
    tf::quaternionMsgToTF(odom_msg.pose.pose.orientation,tfq);

    tf::Matrix3x3(tfq).getEulerYPR(y,p,r);

    // TODO: Check signs?
    //orientation
    current_pitch.data= p;
    current_roll.data= r;
    current_yaw.data= y;
    current_depth.data= odom_msg.pose.pose.position.z;
    current_x.data= odom_msg.pose.pose.position.x;
    current_y.data= odom_msg.pose.pose.position.y;

    //Velocity
    current_u.data= odom_msg.twist.twist.linear.x;
    current_v.data= odom_msg.twist.twist.linear.y;
    current_w.data= odom_msg.twist.twist.linear.z;
    current_p.data= odom_msg.twist.twist.angular.x;
    current_q.data= odom_msg.twist.twist.angular.y;
    current_r.data= odom_msg.twist.twist.angular.z;
}

int main(int argc, char** argv){

  std::string node_name = "odom_listener";
  ros::init(argc, argv, node_name);

  ros::NodeHandle node;

  std::string odom_topic_;
  std::string dvl_topic_;
  double freq;

  ros::NodeHandle node_priv("~");
  node_priv.param<std::string>("odom_topic", odom_topic_, "/sam/dr/local/odom/filtered");
  node_priv.param<std::string>("dvl_topic", dvl_topic_, "/sam/core/dvl");
  node_priv.param<double>("loop_freq", freq, 10);

  //initiate subscribers
  ros::Subscriber dvl_sub = node.subscribe(dvl_topic_, 1, DVLCallback);
  ros::Subscriber odom_sub = node.subscribe(odom_topic_, 1, OdomCallback);

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
  ros::Publisher feedback_alt = node.advertise<std_msgs::Float64>("alt", freq);

  ros::Rate rate(10.0);
  while (node.ok()){
    
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


    ROS_INFO_THROTTLE(1.0, "[ odom_listener ] roll: %f, pitch: %f, yaw: %f, depth: %f, x: %f, y: %f, u: %f, v: %f, w: %f, p: %f, q: %f, r: %f alt:%f", current_roll.data,current_pitch.data,current_yaw.data,current_depth.data, current_x.data, current_y.data, current_u.data, current_v.data, current_w.data, current_p.data, current_q.data, current_r.data, current_alt.data);

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
};
