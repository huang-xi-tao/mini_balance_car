#pragma once

#include "robot_likeAscento.hpp"
#include "Controller.hpp"
#include "Jump.hpp"
#include "Transform.hpp"
class Planner_Class
{
public:
    Planner_Class(Robot_Class &robot, Controller_Class &ctrl) : robot(robot), ctrl(ctrl),
                                                                cmd(robot.cmd),
                                                                jumpHandle(robot, ctrl),
                                                                wdg(robot.wdg),
                                                                pitch_limit_height((robot.param_ptr->core.MIN_HEIGHT + (robot.param_ptr->core.MAX_HEIGHT - robot.param_ptr->core.MIN_HEIGHT) * 0.3f)),
                                                                upHandle(robot, ctrl),
                                                                downHandle(robot, ctrl)
    {
    }
    void wdg_check_update(float dt);
    void plan_start(void);
    void transform_handler(const uint8_t input);
    void plan_update(float dt);
    void wheel_plan(const float dt, float forward_d_val);
    void wheel_cmd_reset(const float forward_err = 0, const float yaw_err = 0);
    void yaw_cmd_reset(void);
    void wheel_cmd(float forward_input, float rotation_input, const float dt);
    void height_cmd(float height_input, float dt);
    void pitch_cmd(float height_input, float dt);
    void roll_cmd(const float input, const float dt);
    void leg_len_cmd(const float cmd, const float d_cmd);
    void tail_cmd(const float input, const float dt);

    void Clear_Pos_Err()
    {
        pos_rec = pos;
    }

    uint8_t last_flag = 1;
    const float pitch_limit_height;
    const float pitch_limit_angle = 0.95f;
    bool reach_ground = 0;
    bool kill_flag = 0;

    Jump_Handle jumpHandle;
    Transform_Up_Handle upHandle;
    Transform_Down_Handle downHandle;

private:
    Robot_Class &robot;
    Controller_Class &ctrl;
    Cmd &cmd;
    Watchdog &wdg;
    uint8_t forward_push_status;
    float pos_err_limit = 0.5f, err_cnt = 0, err_rec = 0; /*record the pos when no input*/
    ;
    float pos_rec = 0;
    float last_pitch_val;
    float pos = 0;
    float pos_cmd_stop = 0;
};
