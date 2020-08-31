#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Header.h"

#include "ps_joystick.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "util.h"
#ifdef __cplusplus
}
#endif

#include <cmath>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

static char const* const DEV0 = static_cast<char const*>("/dev/input/js0");
const std::string JOY_ID = "/teleop/intdash_joy";

static unsigned char packet[2048];
static unsigned char buffer[2048];
static char const* const json1 = static_cast<char const*>("{\"header\":{\"seq\":");
static char const* const json2 = static_cast<char const*>(",\"stamp\":{\"secs\":");
static char const* const json3 = static_cast<char const*>(",\"nsecs\":");
static char const* const json4 = static_cast<char const*>("},\"frame_id\":\"intdash\"},");
static char const* const json5 = static_cast<char const*>( "\"axes\":[");
static char const* const json6 = static_cast<char const*>("],\"buttons\":[");

static bool g_mainloop_term = false;
static unsigned char g_seq = 0;

static void
signal_handler(int)
{
    g_mainloop_term = true;
}

static bool
register_signal_handler()
{
    struct sigaction sig_action;

    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = signal_handler;
    if (sigaction(SIGINT, &sig_action, NULL) == -1) {
        PRINT_ERROR("sigaction().. Error, <%s>", strerror(errno));
        g_mainloop_term = true;
        return false;
    }

    return true;
}


static int
ps_joystick_set_initial_value(ps_joystick_t* rcjs)
{
    if (rcjs == NULL) {
        PRINT_ERROR("rcjs is NULL");
        return PSJS_RESULT_Failure;
    }

    rcjs->fd = -1;
    rcjs->axis = NULL;
    rcjs->num_of_axis = 0;
    rcjs->button = NULL;
    rcjs->num_of_button = 0;

    return PSJS_RESULT_Success;
}


ps_joystick_t*
ps_joystick_create()
{
    ps_joystick_t* rcjs = NULL;

    rcjs = (ps_joystick_t*)malloc(sizeof(ps_joystick_t));
    if (rcjs == NULL) {
        PRINT_ERROR("malloc().. Error, <size:%lo>", sizeof(ps_joystick_t));
        return NULL;
    }

    if (ps_joystick_set_initial_value(rcjs) == PSJS_RESULT_Failure) {
        PRINT_ERROR("ps_joystick_set_initial_value().. Error");
        free(rcjs);
        return NULL;
    }

    return rcjs;
}



int ps_joystick_init(ps_joystick_t * rcjs, char const * device_fname){
    rcjs->fd = open(DEV0, O_RDONLY);
    if (rcjs->fd == -1){
        PRINT_ERROR("open().. Error, <%s>", device_fname);
        return PSJS_RESULT_Failure;
    }
   
    rcjs->num_of_axis = 0;

    if (ioctl(rcjs->fd, JSIOCGAXES, &rcjs->num_of_axis) == -1) {
        // error handling
        PRINT_ERROR("ioctl(JSIOCGAXES).. Error, <%s>", strerror(errno));
        return PSJS_RESULT_Failure;
    }

    if (ioctl(rcjs->fd, JSIOCGBUTTONS, &rcjs->num_of_button) == -1) {
        PRINT_ERROR("ioctl(JSIOCGBUTTONS).. Error, <%s>", strerror(errno));
        return PSJS_RESULT_Failure;
    }

    if (ioctl(rcjs->fd, JSIOCGNAME(80), rcjs->device_name) == -1) {
        PRINT_ERROR("ioctl(JSIOCGNAME).. Error, <%s>", strerror(errno));
        return PSJS_RESULT_Failure;
    }

    rcjs->axis = (int16_t*)calloc(rcjs->num_of_axis, sizeof(int16_t));
    if (rcjs->axis == NULL) {
        PRINT_ERROR("calloc().. Error, <size:%lo>",
                    rcjs->num_of_axis * sizeof(int16_t));
        return PSJS_RESULT_Failure;
    }

    rcjs->button = (int16_t*)calloc(rcjs->num_of_button, sizeof(int16_t));
    if (rcjs->button == NULL) {
        PRINT_ERROR("calloc().. Error, <size:%lo>",
                    rcjs->num_of_button * sizeof(int16_t));
        return PSJS_RESULT_Failure;
    }

    if (fcntl(rcjs->fd, F_SETFL, O_NONBLOCK) != PSJS_RESULT_Success) {
        PRINT_ERROR("fcntl(F_SETFL).. Error, <%s>", strerror(errno));
        return PSJS_RESULT_Failure;
    }

    close(rcjs->fd);
    return PSJS_RESULT_Success;
}

static int joystick_get_input(ps_joystick_t *js, FILE *fp){

    uint32_t const button_data_size =
      js->num_of_button * sizeof(js->button[0]);
    uint32_t const axis_data_size =
      js->num_of_axis * sizeof(js->axis[0]);
    uint8_t* packet = new uint8_t[button_data_size + axis_data_size];

    try {
        size_t read_data_size = fread(
          reinterpret_cast<char*>(packet), sizeof(char), button_data_size + axis_data_size, fp);
        if (read_data_size > 0) {
            if (read_data_size != button_data_size + axis_data_size) {
                delete[] packet;
                PRINT_ERROR("Invalid data size: expect %ud, read %ld", button_data_size+axis_data_size, read_data_size);
                return false;
            }

            memcpy(js->button, packet, button_data_size);
            memcpy(js->axis, packet + button_data_size, axis_data_size);
            for (unsigned int i = 0; i < js->num_of_axis; ++i) {
                js->axis[i] -= i < 16 ? 128 : 512;
            }
        }
    } catch (const std::exception& e) {
        PRINT_ERROR("Exception: %s", e.what());
        return false;
    }

    delete[] packet;

    return true;

}

bool file_exists(const char* path)
{
    struct stat st;

    if (stat(path, &st) != 0) {
        return false;
    }
    return true;
}

void wait_path_creation(const std::string &path){
    ros::Rate wait(1);
    while(ros::ok()){
        bool tmp = file_exists(path.c_str());
        if (tmp)break;
        wait.sleep();
    }
}

void* read_controller_data(void *void_rcjs){
    ps_joystick_t* rcjs = reinterpret_cast<ps_joystick_t*>(void_rcjs);
    rcjs->fp = fopen(DEV0, "rb");

    const std::string ps3joy_path="/tmp/ps3joy.bin";
    wait_path_creation(ps3joy_path);
 
    int cnt=0;
    while(!g_mainloop_term){
        FILE *fp = fopen(ps3joy_path.c_str(), "rb");

        bool res = joystick_get_input(rcjs, fp);
        if (!res){
            PRINT_ERROR("ps_joystick_get_input failed..");
            break;
        }
        fclose(fp);
        usleep(1000);
    }
}

void encode(std_msgs::Header & h, int num_of_axis, std::vector<float> & axes, int num_of_button, std::vector<int> & buttons, 
            FILE *fp, unsigned int sec, unsigned int nsec){
    int p = 0;
    strcpy(reinterpret_cast<char*>(&buffer[p]), json1);
    p += strlen(json1);

    p += sprintf(reinterpret_cast<char*>(&buffer[p]), "%d", h.seq);

    strcpy(reinterpret_cast<char*>(&buffer[p]), json2);
    p += strlen(json2);

    p += sprintf(reinterpret_cast<char*>(&buffer[p]), "%d", h.stamp.sec);

    strcpy(reinterpret_cast<char*>(&buffer[p]), json3);
    p += strlen(json3);

    p += sprintf(reinterpret_cast<char*>(&buffer[p]), "%d", h.stamp.nsec);

    strcpy(reinterpret_cast<char*>(&buffer[p]), json4);
    p += strlen(json4);

    // axes
    strcpy(reinterpret_cast<char*>(&buffer[p]), json5);
    p += strlen(json5);

    for(int i = 0;i < num_of_axis;i++){
        if (i)buffer[p++] = ',';
        p += sprintf(reinterpret_cast<char*>(&buffer[p]), "%.8f", axes[i]);
    }

    strcpy(reinterpret_cast<char*>(&buffer[p]), json6);
    p += strlen(json6);

    for(int i = 0;i < num_of_button;i++){
        if (i)buffer[p++] = ',';
        p += sprintf(reinterpret_cast<char*>(&buffer[p]), "%d", buttons[i]);
    }

    buffer[p++] = ']';
    buffer[p++] = '}';

    // message type
    packet[0] = 1;
    // length
    unsigned int vlen = 10 + 1 + JOY_ID.size() + p;
    packet[1] = static_cast<unsigned char>(vlen);
    packet[2] = static_cast<unsigned char>(vlen >> 8);
    packet[3] = static_cast<unsigned char>(vlen >> 16);

    // sec
    packet[4] = static_cast<unsigned char>(sec);
    packet[5] = static_cast<unsigned char>(sec>>8);
    packet[6] = static_cast<unsigned char>(sec>>16);
    packet[7] = static_cast<unsigned char>(sec>>24);

    packet[8] = static_cast<unsigned char>(nsec);
    packet[9] = static_cast<unsigned char>(nsec>>8);
    packet[10] = static_cast<unsigned char>(nsec>>16);
    packet[11] = static_cast<unsigned char>(nsec>>24);


    // dtype
    packet[12] = 0x1d; // string
    // seq
    packet[13] = g_seq;
    g_seq = (g_seq + 1) % 256;

    packet[14] = static_cast<unsigned char>(JOY_ID.size());
    strncpy(reinterpret_cast<char*>(&packet[15]), JOY_ID.c_str(), JOY_ID.size());
    strncpy(reinterpret_cast<char*>(&packet[15 + packet[14]]), reinterpret_cast<char*>(buffer), p);

    int fd = fileno(fp);
    int write_len = 15 + JOY_ID.size() + p;
    if (write(fd, reinterpret_cast<const void*>(packet), write_len) != write_len){
        PRINT_ERROR("failed to write fifo: joy_string");
        std::exit(1);
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "ps3joy_reader_node");
    ros::NodeHandle nh;

    pthread_t read_controller_data_thread;

    ps_joystick_t* rcjs = ps_joystick_create();
    if (rcjs == NULL){
        PRINT_ERROR("ps_joystick_create() failed.. Erro, <%s>", strerror(errno));
    }

    wait_path_creation(DEV0);

    int res = ps_joystick_init(rcjs, DEV0);
    if (res == PSJS_RESULT_Failure){
        PRINT_ERROR("ps_joystick_init failed.. Error, <%s>", strerror(errno));
        return -1;
    }

    /*** SIGNAL ***/
    if (!register_signal_handler()) {
        PRINT_ERROR("register_signal_handler().. Error");
        return -1;
    }

    if (pthread_create(
          &read_controller_data_thread, NULL, read_controller_data, reinterpret_cast<void*>(rcjs)) !=
        0) {
        PRINT_ERROR("pthread_create().. Error");
        return -1;
    }

    ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("/teleop/intdash_joy", 100);
    bool use_custom_logger=false;
    if (!ros::param::get("~use_custom_logger", use_custom_logger)){
        ROS_ERROR("use_custom_logger doesn't exist");
        std::exit(1);
    }

    FILE *fp;
    if (use_custom_logger){
        std::string fifo_path;
        if (!ros::param::get("~fifo_path", fifo_path)){
            PRINT_ERROR("fifo_path doesn't exist");
            std::exit(1);
        }
        wait_path_creation(fifo_path);
    
        int fd = open(fifo_path.c_str(), O_WRONLY);
        if (fd == -1){
            PRINT_ERROR("cannot open fifo_path");
            std::exit(1);
        }

        fp = fopen(fifo_path.c_str(), "w");
        if (fp == NULL){
            PRINT_ERROR("cannot open fifo_path");
            std::exit(1);
        }
        setvbuf(fp , NULL, _IONBF, 0);
    }

    int seq = 0;
    while(!g_mainloop_term){
        ros::WallTime wall = ros::WallTime::now();
        std_msgs::Header h;
        h.seq = seq++;
        h.stamp.sec = wall.sec;
        h.stamp.nsec = wall.nsec;

        std::vector<int> buttons(rcjs->num_of_button);
        for(int i = 0;i < rcjs->num_of_button;i++){
            buttons[i] = rcjs->button[i];
        }
        std::vector<float> axes(rcjs->num_of_axis);
        for(int i = 0;i < rcjs->num_of_axis;i++){
            axes[i] = (static_cast<float>(rcjs->axis[i]) + 0.5) / (i < 16 ? 128 : 512);
            if (std::abs(axes[i]) < 0.1){
                axes[i] = 0;
            }
        }

        struct timespec tp;
        clock_gettime(CLOCK_MONOTONIC_RAW, &tp);

        if (use_custom_logger){
            encode(h, rcjs->num_of_axis, axes, rcjs->num_of_button, buttons, fp, tp.tv_sec, tp.tv_nsec);
        } else {
            sensor_msgs::Joy joy;
            joy.header = h;
            joy.axes = axes;
            joy.buttons = buttons;
            joy_pub.publish(joy);
            
        }
        
        usleep(20000); // 20msec
    }

    pthread_join(read_controller_data_thread, NULL);

    return 0;
}