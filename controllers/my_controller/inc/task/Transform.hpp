#pragma once

#include <stdint.h>
#include "Ramp.hpp"
#include "robot_likeAscento.hpp"
#include "Controller.hpp"

class Transform_Up_Handle
{
public:
    Transform_Up_Handle(Robot_Class &robot, Controller_Class &ctrl) : robot(robot), balance_init(false), ctrl(ctrl)
    {
        init_height = init_head_angle = init_roll = init_wheel_output = transition_time = 0;
        init_tail_angle = robot.param_ptr->core.TAIL_DOWN_POS_FIRST;
    }

    void start(void);
    void update(const float dt);

    bool finish(void)
    {
        return (height_ramp.finish() & tail_ramp.finish());
    }

    float head_angle(const float dest)
    {
        return height_ramp.getResult(init_head_angle, dest);
    }

    float d_head_angle(const float dest)
    {
        return height_ramp.getDResult(init_head_angle, dest);
    }

    float height_change(const float dest)
    {
        return height_ramp.getResult(init_height, dest);
    }

    float roll_change(const float dest)
    {
        return height_ramp.getResult(init_roll, dest);
    }

    float tail_change(const float dest)
    {
        return tail_ramp.getResult(init_tail_angle, dest);
    }

    float d_height(const float dest)
    {
        return height_ramp.getDResult(init_height, dest);
    }

private:
    Robot_Class &robot;
    Controller_Class &ctrl;

public:
    float init_height;
    float init_head_angle;
    float init_roll;
    float init_tail_angle;

    uint8_t phase = 0; // 0 起立， 1 收尾巴， 2降到指定位置

    bool balance_init;
    float init_wheel_output;
    float transition_time;

    Ramp<float> pitch_ramp;
    Ramp<float> height_ramp;
    Ramp<float> tail_ramp;
};

class Transform_Down_Handle
{
public:
    float stamp = 0;

    Transform_Down_Handle(Robot_Class &robot, Controller_Class &ctrl) : robot(robot), ctrl(ctrl)
    {
        this->reversible = false;
        transition_time = 0;
        retract_time = 0;
        phase = 0;
    };

    void start(void);
    void update(const float dt);

    bool finish(void)
    {
        return height_ramp.finish() && tail_ramp.finish();
    }

    bool pitch_finish(void)
    {
        return pitch_ramp.finish();
    }

    float head_angle_change(const float init)
    {
        return pitch_ramp.get_head_result(init, -0.3);
    }

    float d_head_angle(const float init)
    {
        return height_ramp.finish() ? 0 : -init / transition_time;
    }

    float wheel_output(const float vel_output, const float lqr_output)
    {
        return reversible ? lqr_output : vel_output;
    }

    float height_change(const float init)
    {
        return height_ramp.getResult(init, dest_height);
    }

    float tail_change(const float init)
    {
        return tail_ramp.getResult(init, this->robot.param_ptr->core.TAIL_DOWN_POS);
    }

    void set_height_dest(const float dest)
    {
        dest_height = dest;
    }

    float roll(const float init)
    {
        return height_ramp.getResult(init, robot.posture_ptr->ground_tilt);
    }

    float d_height(const float init)
    {
        return height_ramp.getDResult(init, dest_height);
    }

public:
    bool reversible;
    uint8_t phase = 0; // 0 起立 1 收腿 2 蹲下
    float dest_height;

    Ramp<float> height_ramp;
    Ramp<float> pitch_ramp;
    Ramp<float> tail_ramp;

private:
    Robot_Class &robot;
    Controller_Class &ctrl;

    float retract_time;

    float transition_time;
};
