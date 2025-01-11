#include "Init.hpp"
#include "stdio.h"

// 功能1
void Init_Handle::Start()
{
    init = false;
    this->started = true;
    this->reset_trial = 0;

    this->init_d_len = init_d_len;
    this->init_d_ang = init_d_ang;

    robot.wdg.clearFlag(FALLOVER_MASK, Watchdog::CRITICAL);
    robot.wdg.clearFlag(LEG_OVERLOAD, Watchdog::SEVERE);
    robot.wdg.clearFlag(RESET_FAIL, Watchdog::CRITICAL);
    robot.Clear_Robot_Flag();

    l_step = UPDATE_START;
    r_step = UPDATE_START;

    this->reset_trial++;

    if (this->reset_trial > 1)
    {
        this->reset_trial = 0;
        robot.wdg.setFlag(RESET_FAIL, Watchdog::CRITICAL);
        return;
    }
}

// 功能3
void Init_Handle::Update(const float dt)
{
    if (this->init)
    {
        reset_time += dt;
        if (reset_time > 0.5f)
            this->reset_trial = 0;
    }
    else
    {
        len_ramp.update(dt);
        // if(robot->leg_len < robot->param.core.LEG_LEN_RETRACT + 0.02f) return;
        float ang_err_l = mabs(robot.legL_ptr->angle),
              ang_err_r = mabs(robot.legR_ptr->angle);
        ang_ramp.update(dt);
        if (len_ramp.finish() && ang_ramp.finish())
        {
            if (this->reset_mode == Robot_Class::STAND)
            {
                ctrl.transform(Robot_Class::TRANSFORM_UP);
            }

            this->init = true;
            robot.wdg.clearFlag(FALLOVER_MASK, Watchdog::CRITICAL);
        }
    }

    ctrl.left_out.Clear_Output();
    ctrl.right_out.Clear_Output();
    ctrl.leg_len_update(robot.legR_ptr, this->right_leg_len(), ctrl.right_out, dt);
    ctrl.leg_angle_update(robot.legR_ptr, this->right_leg_angle(init_ang, ctrl.set_tilt), ctrl.set_d_tilt, ctrl.right_out, dt);

    ctrl.leg_len_update(robot.legL_ptr, this->left_leg_len(), ctrl.left_out, dt);
    ctrl.leg_angle_update(robot.legL_ptr, this->left_leg_angle(init_ang, ctrl.set_tilt), ctrl.set_d_tilt, ctrl.left_out, dt);
}

void Init_Handle::Package_Run(const float dt)
{
    switch (package_status)
    {
    case INIT_UPDATE:
        Update(dt);
        break;
    case INIT_RESET:
        Start();
        package_status = INIT_UPDATE;
        break;
    }
}
