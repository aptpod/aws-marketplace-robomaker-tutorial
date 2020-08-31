#include "ros/ros.h"
#include "teleop_arm_node/teleop_arm_node_define.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <cassert>

const double grip_init = 0.0;
const double initial_pos[5] = {0.0, 0.0, -1.52, 1.13, 0.6};
const int RETURN_CLOCKS = 200;

namespace ros { class NodeHandle; }

double to_rad(double d) {
  return d * 3.1415927 / 180;
}

double limit(double value, double min, double max) {
  assert(min <= max);
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  }
  return value;
}

double stick_threshhold(double value) {
  if (-0.1 < value && value < 0.1) {
    return 0.0;
  }
  return value;
}

class JoyToTwist
{
public:
  JoyToTwist(ros::NodeHandle* nh, ros::NodeHandle* nh_param);
  void initJoint(ros::NodeHandle* nh);


private:
  void joyCallback(const sensor_msgs::Joy& joy);

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher grip_pub_;
  ros::Publisher joint_pub_;
  ros::Subscriber joy_sub_;

  double grip_old;
  double joint_old[5];
  bool return_mode;
  int return_clock_elapsed;
  double return_speed[5];
};

JoyToTwist::JoyToTwist(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  uint32_t i;
  grip_pub_ = nh->advertise<std_msgs::Float64MultiArray>("/gripper_position", 1);
  joint_pub_ = nh->advertise<std_msgs::Float64MultiArray>("/joint_trajectory_point", 1);
  joy_sub_ = nh->subscribe("/teleop/intdash_joy", 100, &JoyToTwist::joyCallback, this);

  grip_old = grip_init;
  
  for(int i = 0; i < 5; i++) {
    joint_old[i] = initial_pos[i];
  }
  
  return_mode = false;
}

void JoyToTwist::joyCallback(const sensor_msgs::Joy& joy)
{
//  geometry_msgs::Twist vel;
  std_msgs::Float64MultiArray grip;
  std_msgs::Float64MultiArray joint;
  grip.data.resize(1);
  joint.data.resize(5);

  double grip_max = 0.02f;
  double grip_min = -0.015f;

// joint
  if (joy.buttons[3] == 1 && !return_mode) {
    return_mode = true;
    return_clock_elapsed = 0;
    for(int i = 0; i < 5; i++) {
      return_speed[i] = (initial_pos[i] - joint_old[i]) / RETURN_CLOCKS;
    }
  }
  
  if (return_mode) {
    for (int i = 0; i < 5; i++) {
      joint_old[i] += return_speed[i];
      grip_old += (grip_init - grip_old) / (RETURN_CLOCKS - return_clock_elapsed);
    }
    return_clock_elapsed++;
    if (return_clock_elapsed == RETURN_CLOCKS) {
      for (int i = 0; i < 5; i++) {
        joint_old[i] = initial_pos[i];
      }
      grip_old = grip_init;
      return_mode = false;
    }
    grip.data[0] = grip_old;
    for (int i = 0; i < 5; i++) {
      joint.data[i] = joint_old[i];
    }
    joint_pub_.publish(joint);
    grip_pub_.publish(grip);
    return;
  }

  // ID 11
  if (joy.buttons[11] == 1){ // PS3R1Button
    const double max_angular_speed = 0.8;
  
    // For ID11 (Joint)
    double id11 = joint_old[1];
    double diff_id11 = 0.0;
    if (joy.buttons[7] == 1){ // PS3LeftButton
      diff_id11 = to_rad(max_angular_speed);
    } else if (joy.buttons[5] == 1){ // PS3RightButton
      diff_id11 = to_rad(-max_angular_speed);  
    }
    id11 += diff_id11;
    joint_old[1] = limit(id11, to_rad(-180), to_rad(180));
  
    // For ID12 (Joint)
    double id12 = joint_old[2];
    double diff_id12 = 0.0;
    if (joy.buttons[4] == 1){ // PS3UpButton
      diff_id12 = to_rad(max_angular_speed);
    } else if (joy.buttons[6] == 1){ // PS3DownButton
      diff_id12 = to_rad(-max_angular_speed);  
    }
    id12 += diff_id12;
    joint_old[2] = limit(id12, to_rad(-100), to_rad(75));
    
    // For ID13 (Joint)
    double id13 = joint_old[3];
    double diff_id13 = to_rad(stick_threshhold(joy.axes[1]) * max_angular_speed); // PS3StickLeftV -1.0~+1.0
    joint_old[3] = limit(id13 + diff_id13, to_rad(-90), to_rad(80));
    
    // For ID14 (Joint)
    double id14 = joint_old[4];
    double diff_id14 = to_rad(stick_threshhold(joy.axes[3]) * max_angular_speed); // PS3StickRightV -1.0~+1.0
    joint_old[4] = limit(id14 + diff_id14, to_rad(-90), to_rad(90));
  }
  
  // grip
  int32_t grip_hold = joy.buttons[14]; // PS3CrossButton
  int32_t grip_release = joy.buttons[13]; // PS3CircleButton
  double grip_diff = 0.0;
  if(grip_hold == 1 && grip_release == 1) {
  } else if(grip_hold == 1) {
    grip_diff = -0.0005;
  } else if(grip_release == 1) {
    grip_diff = 0.001;
  }
  grip_old = limit(grip_old + grip_diff, grip_min, grip_max);

  // publish
  grip.data[0] = grip_old;
  for( uint32_t i = 0; i < 5; i++ ) joint.data[i] = joint_old[i];

  grip_pub_.publish(grip);
  joint_pub_.publish(joint);
}

void JoyToTwist::initJoint(ros::NodeHandle* nh)
{
  std::string control_mode;
  nh->getParam("/om_with_tb3/teleop_arm_node/control_mode", control_mode);
  
  if(control_mode != "KEY") {
    return;
  }
  
  std_msgs::Float64MultiArray joint;
  joint.data.resize(5);
  for( uint32_t i = 0; i < 5; i++ ) joint.data[i] = initial_pos[i];
  
  ros::Rate loop_rate(1);
  for(uint32_t i = 0; i < 100; i++) {
    joint_pub_.publish(joint);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_arm_node");
  ros::NodeHandle nh(""), nh_param("~");
  JoyToTwist joytotwist(&nh, &nh_param);
  joytotwist.initJoint(&nh);

  ros::spin();
}
