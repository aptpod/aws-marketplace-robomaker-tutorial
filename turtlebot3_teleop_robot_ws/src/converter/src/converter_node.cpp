#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <pthread.h>

#include <sys/stat.h>

#include <ros/ros.h>
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf2/utils.h"

#include <boost/bind.hpp>

#include "basetime/basetime.h"

typedef bt_err (*bt_get_relative_time_t)(unsigned int *sec, unsigned int *nsec);

const double rad2deg = 180 / M_PI;

unsigned char seq = 0;

void encode(const std::string &name, const double val, FILE *fp, bt_get_relative_time_t bt_get_relative_time,
            pthread_mutex_t *m){
  int r;
  r = pthread_mutex_lock(m);
  if (r != 0) {
    ROS_ERROR("can not lock %d", r);
    std::exit(1);
  }

  unsigned char type = 1;
  if (fwrite(&type, sizeof(unsigned char), 1, fp) != 1 ){
    ROS_ERROR("failed to write fifo: type");
    std::exit(1);
  }
  unsigned int vlen = 8 + 10 + 1 + name.size();
  unsigned char vlen_byte[3] = {static_cast<unsigned char>(vlen & 0x0000FF), 
  static_cast<unsigned char>((vlen & 0x00FF00) >> 8), 
  static_cast<unsigned char>((vlen & 0xFF0000) >> 16)};
  if (fwrite(vlen_byte, sizeof(unsigned char), 3, fp) != 3){
    ROS_ERROR("failed to write fifo: vlen");
    std::exit(1);
  }

  unsigned int sec, nsec;
  bt_get_relative_time(&sec, &nsec);
    if (fwrite(&sec, sizeof(unsigned int), 1, fp) != 1){
    ROS_ERROR("failed to write fifo: sec");
    std::exit(1);
  }
  if (fwrite(&nsec, sizeof(unsigned int), 1, fp) != 1){
    ROS_ERROR("failed to write fifo: nsec");
    std::exit(1);
  }
  
  // must use mutex to get seq
  unsigned char dtype = 0x1e;
  if (fwrite(&dtype, sizeof(unsigned char), 1, fp) != 1){
    ROS_ERROR("failed to write fifo: dtype");
    std::exit(1);
  }
  if (fwrite(&seq, sizeof(unsigned char), 1, fp) != 1){
    ROS_ERROR("failed to write fifo: seq");
    std::exit(1);
  }
  seq = (seq + 1) % 256;
  
  unsigned char name_len = name.size();
  if (fwrite(&name_len, sizeof(unsigned char), 1, fp) != 1){
    ROS_ERROR("failed to write fifo: name_len");
    std::exit(1);
  }
  if (fwrite(name.c_str(), sizeof(char), name_len, fp) != name_len){
    ROS_ERROR("failed to write fifo: name");
    std::exit(1);
  }
  
  if (fwrite(&val, sizeof(double), 1, fp) != 1){
    ROS_ERROR("failed to write fifo: val");
    std::exit(1);
  }

  r = pthread_mutex_unlock(m);
  if (r != 0) {
    ROS_ERROR("can not unlock %d", r);
    std::exit(1);
  }
}

void CallbackScan(const sensor_msgs::LaserScan::ConstPtr & msg, FILE* fp, bt_get_relative_time_t bt_get_relative_time, pthread_mutex_t *m){
  double scan_range_minimum = 1000;
  for(int i = 0;i < msg->ranges.size();i++){
    double a = -(msg->angle_min + msg->angle_increment * i) * rad2deg;
    if (msg->ranges[i] > 0.0 && scan_range_minimum > msg->ranges[i]){
      scan_range_minimum = msg->ranges[i];
    }
    encode("scan_range", msg->ranges[i], fp, bt_get_relative_time, m);
    encode("scan_angle", a, fp, bt_get_relative_time, m);
  }
  encode("scan_range_minimum", scan_range_minimum, fp, bt_get_relative_time, m);
}

void CallbackOdom(const nav_msgs::Odometry::ConstPtr& msg, FILE* fp, bt_get_relative_time_t bt_get_relative_time, pthread_mutex_t *m){
  double yaw, pitch, roll;
  tf2::getEulerYPR(msg->pose.pose.orientation, yaw, pitch, roll);
  encode("odom_ori_yaw", -(yaw * rad2deg), fp, bt_get_relative_time, m);
}

bool existFile(const char* path)
{
    struct stat st;

    if (stat(path, &st) != 0) {
        return false;
    }
    return true;
}


void WaitPathCreation(const std::string &path){
  ros::Rate wait(1);
  while(ros::ok()){
    bool tmp = existFile(path.c_str());
    if (tmp)break;
    wait.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "converter_node");
  ros::NodeHandle nh;

  std::string fifo_path;
  if (!ros::param::get("~fifo_path", fifo_path)){
    ROS_ERROR("fifo_path doesn't exist");
    std::exit(1);
  }
  WaitPathCreation(fifo_path);
  FILE *fp = fopen(fifo_path.c_str(), "wb");
  if (fp == NULL){
    ROS_ERROR("fifo path doen't exist");
    std::exit(1);
  }

  std::string basetime_path;
  if (!ros::param::get("~basetime_path", basetime_path)){
    ROS_ERROR("basetime_path doesn't exist");
    std::exit(1);
  }
  WaitPathCreation(basetime_path);
  std::string basetime_lib_path;
  if (!ros::param::get("~basetime_lib_path", basetime_lib_path)){
    ROS_ERROR("basetime_lib_path doesn't exist");
    std::exit(1);
  }

  void *handle;
  handle = dlopen(basetime_lib_path.c_str(), RTLD_NOW);
  //enum bt_err (*bt_init)(const char* btfile, int create_base);
  typedef bt_err (*bt_init_t)(const char* btfile, int create_base);
  bt_init_t bt_init;
  bt_init = reinterpret_cast<bt_init_t>(reinterpret_cast<long >(dlsym(handle, "bt_init")));
  char *error = dlerror();
  if (error != NULL){
    ROS_ERROR("failed to load bt_init");
    std::exit(1);
  }
  bt_init(basetime_path.c_str(), false);

  bt_get_relative_time_t bt_get_relative_time;
  bt_get_relative_time = reinterpret_cast<bt_get_relative_time_t>(reinterpret_cast<long >(dlsym(handle, "bt_get_relative_time")));
  error = dlerror();
  if (error != NULL){
    ROS_ERROR("failed to load bt_get_relative_time");
    std::exit(1);
  }

  std::string scan_topic_name;
  if (!ros::param::get("~scan_topic_name", scan_topic_name)){
    ROS_ERROR("scan_topic_name doesn't exist");
    std::exit(1);
  }

  std::string odom_topic_name;
  if (!ros::param::get("~odom_topic_name", odom_topic_name)){
    ROS_ERROR("odom_topic_name doesn't exist");
    std::exit(1);
  }

  pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;

  ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>(scan_topic_name, 100, boost::bind(CallbackScan, _1, fp, bt_get_relative_time, &m));
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_name,100,boost::bind(CallbackOdom, _1, fp, bt_get_relative_time, &m));


  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("wait for shutdown");
  ros::waitForShutdown();
  fclose(fp);
  pthread_mutex_destroy(&m);
  return 0;
}