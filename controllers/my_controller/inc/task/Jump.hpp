#pragma once

#include "robot_likeAscento.hpp"
#include "Controller.hpp"
#include "Ramp.hpp"
#include "Init.hpp"

class Robot_Class;

class Jump_Handle
{
public:
    Jump_Handle(Robot_Class &robot, Controller_Class &ctrl);

    bool right_lifted = false;
    bool left_lifted = false;

    void update(const float dt);

    float leg_limit(void)
    {
        return param.core.LEG_MAX_LEN;
    }

    float tilt(float dest)
    {
        return pitch_ramp.getResult(init_tilt, dest);
    }

    void tilt_cmd_reset(void)
    {
        this->init_pitch = robot.posture_ptr->head_pitch;
        this->init_tilt = robot.posture_ptr->tilt; // robot.posture_ptr->tilt;

        robot.Set_Robot_Flag(Robot_Class::OFFGROUND, false);
        ctrl.set_pitch = robot.posture_ptr->head_pitch;
        ctrl.set_tilt = robot.param_ptr->imu_bias.y;

        this->pitch_ramp.reset(0.5);
    }

    float d_tilt(const float dest, const float d_dest)
    {
        if (!pitch_ramp.finish())
            return pitch_ramp.getDResult(init_tilt, dest);
        else
            return d_dest;
    }

    float pitch(float dest)
    {
        return pitch_ramp.getResult(init_pitch, dest);
    }

    float d_pitch(const float dest, const float d_dest)
    {
        if (pitch_ramp.finish())
            return d_dest;
        else
            return pitch_ramp.getDResult(init_pitch, dest);
    }

public:
    float set_height;
    float set_d_height;
    float lift_vel;
    float lift_len;
    int onground_cnt;
    Ramp<float> pitch_ramp;
    Ramp<float> height_recover_ramp;

private:
    Robot_Class &robot;
    Controller_Class &ctrl;

    const WB6_Parameter &param;
    Cmd &cmd;
    float recover_start_height = 0.f;
    Ramp<float> tilt_ramp;
    float init_diff;
    float init_tilt;
    float dest_tilt;
    float init_pitch;
    float init_vel;
    float init_forward_err;
    float parasetHeight, parasetDHeight;
};
