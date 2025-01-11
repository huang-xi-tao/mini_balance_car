#pragma once

#include <stdint.h>
#include <limits>

class Cmd;

namespace Command
{
    enum input_mode_t
    {
        INPUT_POS = 0,
        INPUT_VEL = 1,
    };

    enum
    {
        TRANSFORM_IDLE = 0,
        TRANSFORM_UP = 1,
        TRANSFORM_DOWN = 2,
        DANCE_MODE = 3,
        WALK_MODE = 4,
    };

    class Channel
    {
    public:
        Channel(input_mode_t ctrl_mode = INPUT_POS) : ctrl_mode(ctrl_mode),
                                                      val(0), rev(0), d_val(0), force(0) {}

        input_mode_t ctrl_mode;
        float val;
        int32_t rev;

        float d_val;
        float force;

        void input_rev(float val_in, const float dt);
        void input_limit(float val_in, const float dt,
                         const float up_limit_d, const float low_limit_d, const float up_limit, const float low_limit);
    };
}

class RC_Cmd
{
public:
    RC_Cmd(void);

public:
    float move_forward;
    float move_left;

    float height;
    float head_up;
    float tilt_left;

    uint8_t dance_mode = 0;
    uint8_t head_ctrl_mode;
    uint8_t transform;
    uint8_t sdk_enable;
    uint8_t api_enable;

    bool kill;
    bool lost;
    bool override;

    bool enable_transform;
    uint32_t update_cnt;
    bool connect_flag;
};

class Cmd
{
public:
    Cmd(
        Command::input_mode_t forward = Command::INPUT_VEL,
        Command::input_mode_t yaw = Command::INPUT_VEL,
        Command::input_mode_t height = Command::INPUT_POS,
        Command::input_mode_t pitch = Command::INPUT_VEL,
        Command::input_mode_t roll = Command::INPUT_POS) : lock(false), kill(false), update_cnt(0),
                                                           transform(Command::TRANSFORM_IDLE),
                                                           height(height),
                                                           pitch(pitch),
                                                           roll(roll),
                                                           forward(forward),
                                                           yaw(yaw)
    {
        this->calibration = false;
        this->stamp = 0;
        this->head_ctrl_mode = HEAD_CTRL_STABLE;

        forward_input = rotate_input =
            height_input = pitch_input =
                roll_input = 0;
        head_xyz_cmd[2] = {0};
        head_xyz_angle_cmd[2] = {0};
        crash_flag = 0;
        crash_count = 0;
        crashing_count = 0;
    }

    enum
    {
        HEAD_CTRL_STABLE = 0,
        HEAD_CTRL_ASSIST_BALANCE = 1,
        HEAD_CTRL_ACCLERATION = 2
    };

public:
    bool lock; // set to true before some critical automation app finishes

    bool kill;
    bool calibration;
    uint32_t update_cnt;

    uint8_t transform;
    bool enable_transform;
    uint8_t crash_flag;      // whether the robot crash
    uint32_t crashing_count; // time_buffer for robot crash
    uint32_t crash_count;    // time_buffer for robot recover
    bool crash_restar;
    float cmd_lateral_vel;
    float roll_angle;

    float bias_x;
    float bias_y;

    uint8_t modal;
    uint8_t operate_mode;
    uint8_t sensor_mode;
    bool brake;

    uint8_t head_ctrl_mode;
    float stamp;

    Command::Channel height;
    Command::Channel pitch;
    Command::Channel roll;
    Command::Channel forward;
    Command::Channel yaw;
    Command::Channel tail_angle;

    float forward_input;
    float rotate_input;
    float height_input;
    float pitch_input;
    float roll_input;
    float head_xyz_cmd[3];
    float head_xyz_angle_cmd[3];
};
