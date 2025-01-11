#pragma once

#include "stdint.h"
#include "Cmd.hpp"
#include <webots/keyboard.h>
#include "stdio.h"
#include "robot_likeAscento.hpp"
#include "DIABLO.hpp"
// keyboard class
class remote_cmd
{
public:
    remote_cmd() : robot_mode(0), release_time(0), release_key_flag(false)
    {
    }

    enum robot_key
    {
        /*Motion instruction*/
        FOWARD = 87,     // W
        BACK = 83,       // S
        LEFT = 65,       // A
        RIGHT = 68,      // D
        MASS_HIGHT = 56, // 8
        MASS_DOWM = 53,  // 5
        // JUMP = 32,       // SPACE
        // JUMP_STEP = 54,  // 6

        /*head instrunction*/
        HEAD_UP = 315,    // up arrow
        HEAD_DOWM = 317,  // down arrow
        HEAD_LEFT = 314,  // left arrow
        HEAD_RIGHT = 316, // right arrow

        /*other instruction*/
        TRANSFORM_UP = 43,   //+
        TRANSFORM_DOWM = 45, //-
        EMERGENCY_STOP = 4,  // enter
        BRAKE = 51,          // 3
        RESET = 82,          // r
        KILL = 75,           // k
        RESURGENCE = 49      // 1

    };

private:
    enum robomode
    {
        zero = 0,
    };

public:
    uint16_t get_key_status(void)
    {
        return key_status;
    }
    void keyboard_init(void);
    void stop_move(float dt);
    void check_key_release(float dt);
    void keyboard_update(Task_Class &robot_task, float dt);
    void rc_remote_control(remote_cmd *rc_cmd);

private:
    void set_mode(const int16_t mode);
    void straight_turn(const int16_t straight, const int16_t turn, const float dt);

public:
    Robot_Class *robot;
    RC_Cmd rc;
    uint8_t robot_mode;
    bool release_key_flag;
    float release_time;
    int16_t key_status;

private:
    bool rc_check_flag; // check whether the communication is connected
    uint8_t RC_force_flag;
    uint16_t recheck_zone; // 复位检查的死区

    robot_key mode_set;
};

extern remote_cmd key_control;
