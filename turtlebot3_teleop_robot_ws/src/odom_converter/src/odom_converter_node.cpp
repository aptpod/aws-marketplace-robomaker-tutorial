#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <pthread.h>

#include <sys/stat.h>

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "tf2/utils.h"
#include <boost/bind.hpp>

const double rad2deg = 180 / M_PI;

void CallbackOdom(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher & odom_ori_yaw_pub){
  double yaw, pitch, roll;
  tf2::getEulerYPR(msg->pose.pose.orientation, yaw, pitch, roll);
  std_msgs::Float64 odom_ori_yaw;
  odom_ori_yaw.data = -(yaw * rad2deg);
  odom_ori_yaw_pub.publish(odom_ori_yaw);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_converter_node");
  ros::NodeHandle nh;

  std::string odom_topic_name;
  if (!ros::param::get("~odom_topic_name", odom_topic_name)){
    ROS_ERROR("odom_topic_name doesn't exist");
    std::exit(1);
  }

  ros::Publisher odom_ori_yaw_pub = nh.advertise<std_msgs::Float64>("/odom_ori_yaw", 100);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_name,100,boost::bind(CallbackOdom, _1, odom_ori_yaw_pub));


  ros::spin();
  ROS_INFO("wait for shutdown");
  ros::waitForShutdown();
  return 0;
}
