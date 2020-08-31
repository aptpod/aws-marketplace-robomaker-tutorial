#include <cmath>
#include <dlfcn.h>

#include <sys/stat.h>

#include <ros/ros.h>
#include "sensor_msgs/CompressedImage.h"

#include <boost/bind.hpp>

#include "basetime/basetime.h"

typedef bt_err (*bt_get_relative_time_t)(unsigned int *sec, unsigned int *nsec);

const double rad2deg = 180 / M_PI;

unsigned char seq = 0;

void encode(const std::vector<uint8_t> & data, FILE *fp, bt_get_relative_time_t bt_get_relative_time){
  unsigned char type = 1;
  if (fwrite(&type, sizeof(unsigned char), 1, fp) != 1 ){
    ROS_ERROR("failed to write fifo: type");
    std::exit(1);
  }
  
  unsigned int vlen = data.size() + 10;
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
  unsigned char dtype = 0x12;
  if (fwrite(&dtype, sizeof(unsigned char), 1, fp) != 1){
    ROS_ERROR("failed to write fifo: dtype");
    std::exit(1);
  }
  if (fwrite(&seq, sizeof(unsigned char), 1, fp) != 1){
    ROS_ERROR("failed to write fifo: seq");
    std::exit(1);
  }
  seq = (seq + 1) % 256;
  
  if (fwrite(&data[0], sizeof(uint8_t), data.size(), fp) != data.size()){
    ROS_ERROR("failed to write fifo: name");
    std::exit(1);
  }
}

void CallbackImage(const sensor_msgs::CompressedImage::ConstPtr & msg, FILE*fp, bt_get_relative_time_t bt_get_relative_time){
    encode(msg->data, fp, bt_get_relative_time);
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
  ros::init(argc, argv, "jpeghook");
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

  std::string topic_name;
  if (!ros::param::get("~topic", topic_name)){
    ROS_ERROR("topic doesn't exist");
    std::exit(1);
  }

  ros::Subscriber sub = nh.subscribe<sensor_msgs::CompressedImage>(topic_name, 100, boost::bind(CallbackImage, _1, fp, bt_get_relative_time));

  ros::spin();
  fclose(fp);
  return 0;
}