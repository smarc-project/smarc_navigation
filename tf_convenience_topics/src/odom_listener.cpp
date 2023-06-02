//Listener Node to the Odometry topic to read positions and velocities in the local frame for SAM, Sriharsha Bhat,  29.5.2020
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <smarc_msgs/DVL.h>

//Variable initialization

class OdomListener{

  public:

    std_msgs::Float64 current_roll,current_pitch,current_yaw,current_depth, current_x,current_y, current_u, current_v, current_w, current_p,current_q, current_r, current_alt;
    double r,p,y;
    tf::Quaternion tfq;

    ros::Publisher feedback_pitch, feedback_roll, feedback_yaw, feedback_depth, feedback_x, feedback_y, feedback_z, feedback_alt;
    ros::Publisher feedback_u, feedback_v, feedback_w, feedback_p, feedback_q, feedback_r;

    OdomListener()
    {

      ros::NodeHandle node;

      std::string odom_topic_;
      std::string dvl_topic_;
      double freq;

      ros::NodeHandle node_priv("~");
      node_priv.param<std::string>("odom_topic", odom_topic_, "/sam/dr/local/odom/filtered");
      node_priv.param<std::string>("dvl_topic", dvl_topic_, "/sam/core/dvl");
      node_priv.param<double>("loop_freq", freq, 10);

      // initiate subscribers
      ros::Subscriber dvl_sub = node.subscribe(dvl_topic_, 1, &OdomListener::DVLCallback, this);
      ros::Subscriber odom_sub = node.subscribe(odom_topic_, 1, &OdomListener:: OdomCallback, this);

      // initiate publishers
      feedback_pitch = node.advertise<std_msgs::Float64>("pitch", freq);
      feedback_roll = node.advertise<std_msgs::Float64>("roll", freq);
      feedback_yaw = node.advertise<std_msgs::Float64>("yaw", freq);
      feedback_depth = node.advertise<std_msgs::Float64>("depth", freq);
      feedback_x = node.advertise<std_msgs::Float64>("x", freq);
      feedback_y = node.advertise<std_msgs::Float64>("y", freq);
      feedback_u = node.advertise<std_msgs::Float64>("u", freq);
      feedback_v = node.advertise<std_msgs::Float64>("v", freq);
      feedback_w = node.advertise<std_msgs::Float64>("w", freq);
      feedback_p = node.advertise<std_msgs::Float64>("p", freq);
      feedback_q = node.advertise<std_msgs::Float64>("q", freq);
      feedback_r = node.advertise<std_msgs::Float64>("r", freq);
      feedback_alt = node.advertise<std_msgs::Float64>("altitude", freq);

      ros::spin();
    }

    //Callback to read altitude from DVL
    void DVLCallback(const smarc_msgs::DVL& dvl_msg)
    {
        current_alt.data = dvl_msg.altitude;
        feedback_alt.publish(current_alt);

        // ROS_INFO_THROTTLE(1.0, "[ odom_listener]  Altitude from DVL: %f", dvl_msg.altitude);
    }

    //Callback to read states from odomety
    void OdomCallback(const nav_msgs::Odometry& odom_msg)
    {
        tf::quaternionMsgToTF(odom_msg.pose.pose.orientation,tfq);

        tf::Matrix3x3(tfq).getEulerYPR(y,p,r);

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

        // Publish orientation
        feedback_pitch.publish(current_pitch);
        feedback_roll.publish(current_roll);
        feedback_yaw.publish(current_yaw);
        feedback_depth.publish(current_depth);
        feedback_x.publish(current_x);
        feedback_y.publish(current_y);

        // Publish velocity
        feedback_u.publish(current_u);
        feedback_v.publish(current_v);
        feedback_w.publish(current_w);
        feedback_p.publish(current_p);
        feedback_q.publish(current_q);
        feedback_r.publish(current_r);
    }
};

int main(int argc, char** argv){

  ros::init(argc, argv, "odom_listener");

  OdomListener odom;

  return 0;
};
