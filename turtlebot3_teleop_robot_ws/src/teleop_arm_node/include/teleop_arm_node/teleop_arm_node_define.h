#ifndef TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_DEFINE_H
#define TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_DEFINE_H

class TeleopTwistJoyDefine
{
public:
  enum JoyAxisJS0 {
    LEFT_STICK_HORIZONTAL_0 = 0,
    LEFT_STICK_VERTICAL_0 = 1,
    L2_0 = 2,
    RIGHT_STICK_HORIZONTAL_0 = 3,
    RIGHT_STICK_VERTICAL_0 = 4,
    R2_0 = 5,
  };
  enum ButtonJS0 {
    BUTTON_CROSS = 0,    // X
    BUTTON_CIRCLE = 1,   // 〇
    BUTTON_TRIANGLE = 2, // △
    BUTTON_SQUARE = 3,   // □
    BUTTON_L1 = 4,       // L1
    BUTTON_R1 = 5,       // R1
    BUTTON_L2 = 6,       // L1
    BUTTON_R2 = 7,       // L2
    BUTTON_SELECT = 8,   // SELECT
    BUTTON_START = 9,    // START
    BUTTON_HOME = 10,    // HOME
    BUTTON_LEFTSTICK = 11, //
    BUTTON_RIGHTSTIC = 12, //
    BUTTON_UP = 13,      //
    BUTTON_DOWN = 14,    //
    BUTTON_LEFT = 15,    //
    BUTTON_RIGHT = 16,   //
  };
};

#endif  // TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_DEFINE_H
