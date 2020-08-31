#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <pthread.h>

#include <sys/stat.h>

#include <ros/ros.h>
#include <scan_msgs/Scan.h>
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"

#include <boost/bind.hpp>

const double rad2deg = 180 / M_PI;

void CallbackScan(const sensor_msgs::LaserScan::ConstPtr & msg, const ros::Publisher & scan_pub, 
const ros::Publisher & scan_range_min_pub){
  double scan_range_minimum = 1e300;
  for(int i = 0;i < msg->ranges.size();i++){
    double a = -(msg->angle_min + msg->angle_increment * i) * rad2deg;
    if (msg->ranges[i] > 0.0 && scan_range_minimum > msg->ranges[i]){
      scan_range_minimum = msg->ranges[i];
    }
    scan_msgs::Scan scan;
    scan.range = msg->ranges[i];
    scan.angle = a;
    scan_pub.publish(scan); 
  }
  std_msgs::Float64 scan_range_min;
  scan_range_min.data = scan_range_minimum;
  scan_range_min_pub.publish(scan_range_min);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_converter_node");
  ros::NodeHandle nh;

  std::string scan_topic_name;
  if (!ros::param::get("~scan_topic_name", scan_topic_name)){
    ROS_ERROR("scan_topic_name doesn't exist");
    std::exit(1);
  }

  ros::Publisher scan_pub = nh.advertise<scan_msgs::Scan>("converted_scan", 360);
  ros::Publisher scan_range_min_pub = nh.advertise<std_msgs::Float64>("scan_range_min", 10);
  ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>(scan_topic_name, 100, boost::bind(CallbackScan, _1, scan_pub, scan_range_min_pub));


  ros::spin();
  return 0;
}