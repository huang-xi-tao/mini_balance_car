#include "Controller.hpp"
#include "MathFunc.hpp"
#include "stdio.h"

bool Controller_Class::setup(void)
{
    left_angle_pid.kp = right_angle_pid.kp = robot.param_ptr->core.LEG_TILT_KP;
    left_angle_pid.ki = right_angle_pid.ki = robot.param_ptr->core.LEG_TILT_KI;
    left_angle_pid.kd = right_angle_pid.kd = robot.param_ptr->core.LEG_TILT_KD;
    left_angle_pid.max_int = right_angle_pid.max_int = robot.param_ptr->core.LEG_TILT_INT_MAX;
    left_angle_pid.max_out = right_angle_pid.max_out = robot.param_ptr->core.LEG_TILT_OUT_MAX;

    drive_pid.kp = robot.param_ptr->core.DRIVE_KP;
    drive_pid.ki = 0;
    drive_pid.kd = robot.param_ptr->core.DRIVE_KD;
    drive_pid.max_int = 0;
    drive_pid.max_out = robot.legL_ptr->wheel.max_output_torque;

    drive_lock_pid.kp = 5.f;
    drive_lock_pid.ki = 100.f;
    drive_lock_pid.kd = 0.f;
    drive_lock_pid.max_int = 6.f;
    drive_lock_pid.max_out = robot.legL_ptr->wheel.max_output_torque;

    left_len_pid.kp = right_len_pid.kp = robot.param_ptr->core.LEG_LEN_KP;
    left_len_pid.ki = right_len_pid.ki = robot.param_ptr->core.LEG_LEN_KI;
    left_len_pid.kd = right_len_pid.kd = robot.param_ptr->core.LEG_LEN_KD;
    left_len_pid.max_int = right_len_pid.max_int = robot.legL_ptr->knee.max_output_torque / 2;
    left_len_pid.max_out = right_len_pid.max_out = robot.legL_ptr->knee.max_output_torque;

    leg_diff_pid.kp = robot.param_ptr->core.LEG_LEN_DIFF_KP;
    leg_diff_pid.kd = robot.param_ptr->core.LEG_LEN_DIFF_KD;

    roll_pid.kp = robot.param_ptr->core.ROLL_KP;
    roll_pid.ki = robot.param_ptr->core.ROLL_KI;
    roll_pid.kd = robot.param_ptr->core.ROLL_KD;

    return true;
}

void Controller_Class::transform(const Robot_Class::robot_mode_t mode)
{
    if (mode == robot.mode)
        return;

    switch (mode)
    {
    case Robot_Class::CLASH_MODE:
        robot.Set_Robot_Flag(Robot_Class::LEG_ON, false);
        break;
    case Robot_Class::CAR:
        robot.Set_Robot_Flag(Robot_Class::LEG_ON, false);
        break;
    case Robot_Class::TRANSFORM_UP:
        lqr2.reset();
        force.reset();
        roll_pid.reset();
        robot.Set_Robot_Flag(Robot_Class::LEG_ON, true);
        break;
    case Robot_Class::TRANSFORM_REST:
    case Robot_Class::TRANSFORM_DOWN:
        left_angle_pid.reset();
        right_angle_pid.reset();
        break;
    case Robot_Class::STAND:
        drive_lock_pid.reset();
        drive_pid.reset();
        left_len_pid.reset();
        right_len_pid.reset();
        break;
    }
    robot.mode = mode;
}

void Controller_Class::leg_len_update(const Leg2 *leg, const float set_leg_len, Ctrl_Output_Class &output, const float dt)
{
    float out = ctrl_len(leg, set_leg_len, 0, dt);
    output.knee_out = out;
}

void Controller_Class::leg_angle_update(const Leg2 *leg, const float set_leg_angle, const float set_d_leg_angle,
                                        Ctrl_Output_Class &output, const float dt)
{
    float out = 0;

    if (leg->_sideSign == 1)
    {
        out = left_angle_pid.update(set_tilt, set_d_tilt, leg->angle, leg->d_angle, dt);
    }
    else
    {
        out = right_angle_pid.update(set_tilt, set_d_tilt, leg->angle, leg->d_angle, dt);
    }
    output.tilt_out = out;
}

void Controller_Class::leg_tilt_update(const Leg2 *leg, const float set_leg_tilt, const float set_d_leg_tilt,
                                       Ctrl_Output_Class &output, const float dt)
{
    float out = 0;

    if (leg->_sideSign == 1)
    {
        out = left_angle_pid.update(set_tilt, set_d_tilt, leg->tilt, leg->d_tilt, dt);
    }
    else
    {
        out = right_angle_pid.update(set_tilt, set_d_tilt, leg->tilt, leg->d_tilt, dt);
    }
    output.tilt_out = out;
}

void Controller_Class::leg_hybrid_update(const float dt,
                                         const float leg_high_limit,
                                         const float leg_low_limit,
                                         Ctrl_Output_Class &left_output,
                                         Ctrl_Output_Class &right_output)
{
    this->knee_limit_update();

    float cos_roll = cosf(robot.posture_ptr->roll);
    float outL = ctrl_update_lift(Robot_Class::LEFT, force.accl_set_left, cos_roll), // maybe Torque
        outR = ctrl_update_lift(Robot_Class::RIGHT, force.accl_set_right, cos_roll);

    float left_roll, right_roll;
    roll(left_roll, right_roll, dt);
    float wheel_weight = (robot.param_ptr->core.WHEEL_MASS) * GRAVITY;
    float max_left_roll = (outL + 0.6f * wheel_weight) * cos_roll,
          max_right_roll = (outR + 0.6f * wheel_weight) * cos_roll;

    bound(left_roll, max_left_roll);
    bound(right_roll, max_right_roll);

    bound(outL, knee_output_max);
    bound(outR, knee_output_max);
    // printf("roll_outL: %.2f\t, roll_outR: %.2f\n", left_roll, right_roll);

    left_out.knee_out = knee_output(robot.legL_ptr, outL, left_roll, leg_high_limit, leg_low_limit);
    right_out.knee_out = knee_output(robot.legR_ptr, outR, right_roll, leg_high_limit, leg_low_limit);
    // printf("knee_outL: %.2f\t, knee_outR: %.2f\n", left_out.knee_out, right_out.knee_out);
}

float Controller_Class::knee_output(const Leg2 *leg,
                                    float accl_out /*N*/,
                                    const float roll_out /*N*/,
                                    const float high_limit,
                                    const float low_limit)
{
    const float L = robot.param_ptr->core.LEG_LEN_STRAIGHT / 2.f;
    float out = 0;

    out = (roll_out * L - accl_out * L * L * sin(leg->knee_angle));
    // out += this->pos_limit(leg, high_limit, low_limit);
    // printf("roll_out: %.2f \t accl_out: %.2f \t out: %.2f\n", roll_out, accl_out, out);
    return out;
}

float Controller_Class::ctrl_len(const Leg2 *leg, float set_len, float set_d_len, const float dt)
{
    if (set_len > robot.param_ptr->core.LEG_MAX_LEN)
        set_len = robot.param_ptr->core.LEG_MAX_LEN;
    const float __L = robot.param_ptr->core.LEG_LEN_STRAIGHT;
    float sign = 0;
    if (leg->sin_knee_psc < 0.f)
        sign = -1;
    else
        sign = 1;
    float set_knee_angle = sign * 2 * acosf(set_len / __L);
    float set_knee_vel = sign * 2 / sqrtf(__L * __L - set_len * set_len) * set_d_len;
    float knee_angle = (leg->_q1);
    float knee_angle_vel = leg->_dq1;
    if (leg->_sideSign == 1)
    {
        return left_len_pid.update(set_knee_angle, set_knee_vel, knee_angle, knee_angle_vel, dt);
    }
    else
    {
        return right_len_pid.update(set_knee_angle, set_knee_vel, knee_angle, knee_angle_vel, dt);
    }
}

float Controller_Class::ctrl_update_lift(Robot_Class::leg_mark_t leg_mark, float set_accl, const float cos_roll)
{
    const float L = robot.param_ptr->core.LEG_LEN_STRAIGHT / 2;
    float accl_out = 0;
    // float l_psc, r_psc = 0;
    // l_psc = robot.legL_ptr->load_psc;
    // r_psc = 1 - l_psc;

    switch (leg_mark)
    {
    case Robot_Class::LEFT:
        accl_out = set_accl * robot.param_ptr->core.LEG_INERTIA / (4 * L * L);
        // accl_out += ((set_accl)*robot.param_ptr->core.UPPER_MASS * GRAVITY + cos_roll * robot.param_ptr->core.UPPER_MASS * GRAVITY) * l_psc;
        accl_out += set_accl * robot.param_ptr->core.UPPER_MASS * GRAVITY;
        break;
    case Robot_Class::RIGHT:
        accl_out = set_accl * robot.param_ptr->core.LEG_INERTIA / (4 * L * L);
        // accl_out += ((set_accl)*robot.param_ptr->core.UPPER_MASS * GRAVITY + cos_roll * robot.param_ptr->core.UPPER_MASS * GRAVITY) * r_psc;
        accl_out += set_accl * robot.param_ptr->core.UPPER_MASS * GRAVITY;
        break;
    }

    return accl_out;
}

void Controller_Class::set_lqr(const float dt)
{
    lqr2.lqr_start_flag = 1;
    lqr2.update(dt, set_pitch, set_d_pitch, set_tilt, set_d_tilt, cmd.forward.d_val,
                left_out, right_out, pos_forward_err);
}

void Controller_Class::set_lqr_yaw(const float dt, bool init)
{
    lqr_yaw.update(dt, pos_yaw_err, left_out, right_out, set_yaw, init);
}

void Controller_Class::set_wb_lift_accl(const Leg2 *legL, const Leg2 *legR, float &set__left_accl, float &set__right_accl) // set whole body lift accelerate
{
    float set_accl = 0;

    set_accl = force.accl_update(set_leg_len, set_d_leg_len, set_dd_leg_len,
                                 robot.posture_ptr->leg_len, (legL->d_len + legR->d_len) / 2, leg_len_err);
    set__left_accl = set__right_accl = set_accl;
}
