
#include <sys/stat.h>

#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"

const int kJoyAxesSize = 20;
const int kJoyButtonSize = 17; 

// skip p to rosmsg[p] == target
void SkipTo(int &p, char *rosmsg, char target){
    while(rosmsg[p] != target)p++;
}

std_msgs::Header ReadHeader(int &p, char *rosmsg){
    /*
        read
        "header":{"seq":54,"stamp":{"secs":1590031083,"nsecs":909748367},"frame_id":"intdash"}
    */
    // skip "header":{"seq":
    p += 16;

    std_msgs::Header h;
    sscanf(rosmsg + p, "%u", &h.seq);
    SkipTo(p, rosmsg, ',');

    // skip ,"stamp":{"secs":
    p += 17;
    sscanf(rosmsg + p, "%d", &h.stamp.sec);
    SkipTo(p, rosmsg, ',');
    
    // skip ,"nsecs":
    p += 9;
    sscanf(rosmsg+p, "%d", &h.stamp.nsec);
    SkipTo(p, rosmsg, ',');
    
    // skip to ,"frame_id":"
    p += 13;
    // skip intdash
    p += 7;
    h.frame_id = "intdash";

    // skip "}
    p += 2;
    
    return h;
}

template<class T>
std::vector<T> ReadArray(int &p, char *rosmsg, const char * format, const int num){
/*
"axes":[-0.03515625,-0.02734375,-0.02734375,0.01171875,-0.99609375,-0.99609375,-0.99609375,-0.99609375,-0.99609375,-0.99609375,-0.99609375,-0.99609375,-0.99609375,-0.99609375,-0.99609375,-0.99609375,-0.0166015625,0.0166015625,-0.2197265625,-0.9912109375],
"buttons":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}}
*/
    std::vector<T> ret(num);
    // skip to '['
    SkipTo(p, rosmsg, '[');
    p++;
    for(int i = 0;i < num;i++){
        sscanf(rosmsg+p, format, &ret[i]);   
        if (i +1 != num){
            SkipTo(p, rosmsg, ',');
            p++;
        }
    }
    SkipTo(p, rosmsg, ']');
    p++;
    return ret;
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
    ros::init(argc, argv, "teleop_reader_node");
    ros::NodeHandle nh;

    std::string fifo_path;
    if (!ros::param::get("~fifo_path", fifo_path)){
        ROS_ERROR("fifo_path doesn't exist");
        std::exit(1);
    }
    WaitPathCreation(fifo_path);
    FILE *fp = fopen(fifo_path.c_str(), "r");
    if (fp == NULL){
        ROS_ERROR("fifo path doen't exist");
        std::exit(1);
    }

    std::string topic_name;
    if (!ros::param::get("~topic_name", topic_name)){
        ROS_ERROR("topic_name doesn't exist");
        std::exit(1);
    }

    double delay_threshold;
    if (!ros::param::get("~delay_threshold", delay_threshold)){
        ROS_ERROR("delay_threshold doesn't exist");
        std::exit(1);
    }

    const int one_day_sec = 86400;
    ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>(topic_name, 1000);
    ros::Publisher delay_pub = nh.advertise<std_msgs::Float64>("/teleop/delay", 1000);
    
    const int stick_index[4] = {0, 1, 2, 3};
    
    char rostopic[18];// probably enough
    char rosmsg[1000]; // probably enough
    while(ros::ok()){
        try{
            int fread_n;
            unsigned char rxPktData[4];
            fread_n = fread(rxPktData, sizeof(unsigned char), 4, fp);
            unsigned int length = (rxPktData[3] << 16 )  | (rxPktData[2] << 8) | (rxPktData[1]);
            unsigned char dummy[10];
            
            // skip 10 byte
            fread(dummy, sizeof(unsigned char), 10, fp);
            
            unsigned char id_len;
            fread(&id_len, sizeof(unsigned char), 1, fp);

            unsigned int data_len = (length + 4) - id_len - 15;

            fread(rostopic, sizeof(unsigned char), id_len, fp);
            fread(rosmsg, sizeof(unsigned char), data_len, fp);
            rosmsg[data_len] = '\0';

            int p = 1; // skip '{'
            std_msgs::Header h = ReadHeader(p, rosmsg);
            // skip ','
            p++;

            double send_time = h.stamp.toSec();
            double now = ros::WallTime::now().toSec();
            double diff = now - send_time;
            std_msgs::Float64 teleop_delay;
            teleop_delay.data = diff;
            delay_pub.publish(teleop_delay);
            if (now - send_time > delay_threshold && abs(now-send_time) < one_day_sec){
                continue;
            }


            std::vector<float> axes = ReadArray<float>(p, rosmsg, "%f", kJoyAxesSize);            
            // skip ','
            p++;
            

            std::vector<int> buttons = ReadArray<int>(p, rosmsg, "%d", kJoyButtonSize);

            sensor_msgs::Joy joy;
            joy.header = h;
            joy.axes = axes;
            joy.buttons = buttons;

            for(int i = 0;i < 4;i++){
                if (-0.1 <= joy.axes[stick_index[i]] && joy.axes[stick_index[i]] < 0.1){
                    joy.axes[stick_index[i]] = 0;
                }
            }

            joy_pub.publish(joy);

        } catch(char *str){
            ROS_ERROR("%s",str);
        }
    }
    

  return 0;
}