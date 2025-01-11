#include "Jump.hpp"
#include "Cmd.hpp"
#include "MathConst.h"

Jump_Handle::Jump_Handle(Robot_Class &robot, Controller_Class &ctrl) : robot(robot), ctrl(ctrl), param((*robot.param_ptr)), cmd(robot.cmd)
{
    init_diff = init_tilt = dest_tilt = init_pitch = 0;
    set_height = set_d_height = 0;
    lift_len = lift_vel = 0;
    init_vel = init_forward_err = 0;
    pitch_ramp.setVal(2.f, 1);
}

void Jump_Handle::update(const float dt)
{
    this->tilt_ramp.update(dt);
    this->pitch_ramp.update(dt);

    ctrl.force.Set_Force_P(8.5f * ctrl.force.force_p_param);
    ctrl.force.Set_Force_D(8.2f * ctrl.force.force_d_param);

    float setHeight = this->set_height,
          setDHeight = this->set_d_height;

    setHeight = cmd.height.val;
    setDHeight = ctrl.set_dd_leg_len = 0;

    this->set_height = setHeight;
    this->set_d_height = setDHeight;
}
