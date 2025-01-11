#include "Transform.hpp"
#include "stdio.h"
void Transform_Up_Handle::start(void)
{
    this->balance_init = true;

    this->init_height = robot.param_ptr->core.LEG_LEN_RETRACT;
    this->init_roll = robot.posture_ptr->ground_tilt;
    // this->transition_time = (robot.cmd.height.val - init_height) / robot.param_ptr->core.TRANSITION_UP_VEL;
    robot.Set_Robot_Flag(Robot_Class::BALANCE, true);
    height_ramp.reset(1.5f);
    tail_ramp.reset(3.0f);
}

void Transform_Up_Handle::update(const float dt)
{
    // do not start if tilt angle too big
    // if (height_ramp.val == 0 &&
    //     (robot.posture_ptr->tilt > 0.2f || robot.posture_ptr->tilt < -0.2f)
    //     return;
    if (this->phase == 0 || this->phase == 2)
    {
        height_ramp.update(dt);
    }
    else if (this->phase == 1)
    {
        tail_ramp.update(dt);
    }
}

void Transform_Down_Handle::start(void)
{
    this->dest_height = robot.param_ptr->core.LEG_LEN_RETRACT;
    if (phase == 0)
        reversible = true;
    // this->transition_time = (robot.posture_ptr->height - dest_height) / (robot.param_ptr->core.TRANSITION_DOWN_VEL / 0.5f);
    height_ramp.reset(1.0f);
    pitch_ramp.reset(1.0f);
    tail_ramp.reset(3.0f);
    this->retract_time = 0;
    stamp = 0.f;
}

void Transform_Down_Handle::update(const float dt)
{
    stamp += dt;
    // if (robot.posture_ptr->height < robot.param_ptr->core.LEG_LEN_RETRACT + 0.05f)
    // {
    //     this->reversible = false;
    //     this->retract_time += dt;
    // }
    if (stamp > 0.5f) // 延时以供其它操作
    {
        if (phase == 0 || phase == 2)
        {
            height_ramp.update(dt);
        }
        else if (phase == 1)
        {
            tail_ramp.update(dt);
        }
    }
    if (phase == 2 && robot.posture_ptr->height <= robot.param_ptr->core.MIN_HEIGHT + 0.05f)
        reversible = false;
}
