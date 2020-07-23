#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_to_odom_converter");
  std::string source_frame_id, target_frame_id, odom_frame_id;

  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  //  priv_node.param<std::string>("odom_frame", odom_frame_id, "odom");
  priv_node.param<std::string>("source_frame", source_frame_id, "source");
  priv_node.param<std::string>("target_frame", target_frame_id, "target");

  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

  tf::TransformListener listener;

  ros::Rate rate(25.0);
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.waitForTransform(source_frame_id, target_frame_id, ros::Time::now(), ros::Duration(0.5));
      listener.lookupTransform(source_frame_id, target_frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    nav_msgs::Odometry odom;
    // copy pose to odom msg
    odom.header.stamp = transform.stamp_;
    odom.header.frame_id = source_frame_id;
    odom.child_frame_id = target_frame_id;
    geometry_msgs::TransformStamped ts_msg;
    tf::transformStampedTFToMsg(transform, ts_msg);
    odom.pose.pose.position.x = ts_msg.transform.translation.x;
    odom.pose.pose.position.y = ts_msg.transform.translation.y;
    odom.pose.pose.position.z = ts_msg.transform.translation.z;
    odom.pose.pose.orientation = ts_msg.transform.rotation;

    odom_pub.publish(odom);

    rate.sleep();
  }

  return 0;
}
