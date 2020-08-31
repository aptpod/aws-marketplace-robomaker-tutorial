#ifndef __PS_JOYSTICK_H__
#define __PS_JOYSTICK_H__

#include "ps_joystick_defs.h"
#include <linux/joystick.h>
#include <stdint.h>

struct ps_joystick_s
{
    int fd;
    char device_name[80];
    int16_t* axis;
    int num_of_axis;
    int16_t* button;
    int num_of_button;
    FILE*fp;
};
typedef struct ps_joystick_s ps_joystick_t;

#endif /* __PS_JOYSTICK_H__ */