#include "leg_wheel_drive.hpp"
// 功能包调度
void Leg_Wheel_Drive_Class::Package_Run(const float dt)
{
    knee_ctrl(dt);

    wheel_ctrl(dt);

    output_mix(dt);
}

void Leg_Wheel_Drive_Class::knee_ctrl(float dt)
{
    static uint8_t down_steps = 0;
    float leg_min_len = param.core.LEG_MIN_LEN;

    switch (robot.mode)
    {
    case Robot_Class::CAR:
        ctrl.leg_len_update(robot.legL_ptr, leg_min_len, ctrl.left_out, dt);
        ctrl.leg_len_update(robot.legR_ptr, leg_min_len, ctrl.right_out, dt);
        break;
    case Robot_Class::TRANSFORM_UP:
    case Robot_Class::STAND:
        down_steps = 0;

        ctrl.force.accl_set_left = ctrl.force.accl_set_right = ctrl.force.accl_update(ctrl.set_leg_len,
                                                                                      ctrl.set_d_leg_len,
                                                                                      ctrl.set_dd_leg_len,
                                                                                      robot.posture_ptr->leg_len,
                                                                                      (robot.legL_ptr->d_len + robot.legR_ptr->d_len) / 2.0,
                                                                                      ctrl.leg_len_err);
        ctrl.leg_hybrid_update(dt, planner.jumpHandle.leg_limit(), leg_min_len, ctrl.left_out, ctrl.right_out);
        break;

    case Robot_Class::TRANSFORM_DOWN:
        leg_min_len = robot.mode == Robot_Class::TRANSFORM_DOWN ? param.core.LEG_MIN_LEN + 0.03f : param.core.LEG_MIN_LEN;
        ctrl.force.accl_set_left = ctrl.force.accl_set_right = ctrl.force.accl_update(ctrl.set_leg_len,
                                                                                      ctrl.set_d_leg_len,
                                                                                      ctrl.set_dd_leg_len,
                                                                                      robot.posture_ptr->leg_len,
                                                                                      (robot.legL_ptr->d_len + robot.legR_ptr->d_len) / 2.0,
                                                                                      ctrl.leg_len_err);
        ctrl.leg_hybrid_update(dt, planner.jumpHandle.leg_limit(), leg_min_len, ctrl.left_out, ctrl.right_out);

        break;

    case Robot_Class::TRANSFORM_REST:

        break;

    default:
        break;
    }
}

void Leg_Wheel_Drive_Class::wheel_ctrl(float dt)
{
    planner.wheel_plan(dt, robot.cmd.forward.d_val);
    switch (robot.mode)
    {
    case Robot_Class::CAR:
        ctrl.left_out.forward_out = ctrl.right_out.forward_out = ctrl.drive(dt);
        // printf("cmd.d_val: %.2f, \tleft_out:%.2f,\t right_out:%.2f\n", ctrl.cmd.forward.d_val, ctrl.left_out.forward_out, ctrl.right_out.forward_out);
        break;
        // case Robot_Class::TRANSFORM_REST:
        // case Robot_Class::TRANSFORM_DOWN:
        //     drive_out = ctrl.drive(dt);
        //     ctrl.left_out.forward_out = planner.downHandle.wheel_output(drive_out, ctrl.left_out.forward_out);
        //     ctrl.right_out.forward_out = planner.downHandle.wheel_output(drive_out, ctrl.right_out.forward_out);
        //     break;
    }
}

void Leg_Wheel_Drive_Class::output_mix(float dt)
{
    // if(robot.mode != Robot_Class::CAR)
    {
        ctrl.set_lqr_yaw(dt, initHandle.get_init_val());
    }

    if (robot.flag_ptr->balance)
        ctrl.set_lqr(dt);
    else
        ctrl.lqr2.Clear_LQR();
}