#ifndef CONTROLLER_HPP__
#define CONTROLLER_HPP__

#pragma once

#include "robot_likeAscento.hpp"
#include "LQR2.hpp"
#include "LQR_Yaw.hpp"
#include "ForceCtrl.hpp"
#include "PID.hpp"
#include "Filter.hpp"
#include "ctrl_output.hpp"
class Controller_Class
{
public:
    Controller_Class(Robot_Class &robot) : lqr2(robot), lqr_yaw(robot), force(robot),
                                           robot(robot), cmd(robot.cmd)
    {

        set_leg_len = set_d_leg_len = set_dd_leg_len = 0;
        set_pitch = set_d_pitch = 0;

        wheelL_out = wheelR_out = 0;

        knee_output_max = 400;
        knee_output_psc = 1.f;
        this->setup();
        force.setup();

        roll_lpf.init(1000, 100);
        this->pos_forward_err = this->pos_yaw_err = 0;

        force.Set_Force_P(robot.param_ptr->core.IMPEDANCE_KP);
        force.Set_Force_I(0);
        force.Set_Force_D(robot.param_ptr->core.IMPEDANCE_KD);
    }

    float set_leg_len = 0;
    float set_d_leg_len = 0;
    float set_dd_leg_len = 0;
    float set_roll = 0;
    float set_d_roll = 0;
    float set_pitch = 0;
    float set_d_pitch = 0;
    float set_leg_diff = 0;
    float set_d_leg_diff = 0;
    float set_tilt = 0;
    float set_d_tilt = 0;
    float set_yaw = 0;
    float pos_forward_err;
    float pos_yaw_err;
    float pos_yaw_target;
    float roll_err;
    float d_roll_err;
    float leg_len_err;

    float last_pitch_val = 0;

    Ctrl_Output_Class left_out;
    Ctrl_Output_Class right_out;

    float left_lock_knee_angle = 0;
    float left_lock_wheelangle = 0;

    float right_lock_knee_angle = 0;
    float right_lock_wheelangle = 0;
    bool setup(void);

    void output(void)
    {
        robot.Set_Output(left_out, right_out);
    }

    void kill_robot(void)
    {
        left_out.Clear_Output();
        right_out.Clear_Output();
        robot.Set_Output(left_out, right_out);
    }

    void transform(const Robot_Class::robot_mode_t mode);

    float drive(const float dt);

    float drive_vel_lock(const float dt);

    void roll(float &left_roll, float &right_roll, const float dt);

    void leg_len_update(const Leg2 *leg, const float set_leg_len, Ctrl_Output_Class &output, const float dt);

    void leg_angle_update(const Leg2 *leg, const float set_leg_angle, const float set_d_leg_angle,
                          Ctrl_Output_Class &output, const float dt);

    void leg_tilt_update(const Leg2 *leg, const float set_leg_tilt, const float set_d_leg_tilt,
                         Ctrl_Output_Class &output, const float dt);

    void leg_hybrid_update(const float dt, const float leg_high_limit, const float leg_low_limit, Ctrl_Output_Class &left_output, Ctrl_Output_Class &right_output);

    void set_lqr(const float dt);

    void set_lqr_yaw(const float dt, bool init);

    void set_wb_lift_accl(const Leg2 *legL, const Leg2 *legR, float &set__left_accl, float &set__right_accl); // Acceleration Control in force submodule in controller module

    // void set_lift_leg_accl(const Leg2 *leg, Head_estimate head_estimate);

    void knee_limit_update(void)
    {
        this->knee_output_max = 5000;
    }

public:
    float wheelL_out;
    float wheelR_out;
    float knee_output_psc;
    float knee_output_max;

    float ctrl_update_lift(Robot_Class::leg_mark_t leg_mark, float set_accl, const float cos_roll);

    float knee_output(const Leg2 *leg, const float accl_out, const float roll_out,
                      const float high_limit, const float low_limit);

    float ctrl_len(const Leg2 *leg, float set_len, float set_d_len, const float dt);

    float leg_pos_limit(const Leg2 *leg)
    {
        return pos_limit(leg, robot.param_ptr->core.LEG_MAX_LEN, robot.param_ptr->core.LEG_MIN_LEN);
    }

    float pos_limit(const Leg2 *leg, const float high_limit, const float low_limit)
    {
        float output = 0;

        if (leg->len > high_limit)
        {
            output = this->ctrl_len(leg, high_limit, 0, 0);
            output = output < 0 ? 0 : output;
        }
        else if (leg->len < low_limit)
        {
            output = this->ctrl_len(leg, low_limit, 0, 0);
            output = output > 0 ? 0 : output;
        }

        return output;
    }

public:
    PID_Controller drive_pid;
    PID_Controller drive_lock_pid;
    PID_Controller leg_diff_pid;
    PID_Controller roll_pid;

    PID_Controller left_angle_pid;
    PID_Controller right_angle_pid;

    PID_Controller init_len_pid;

    PID_Controller left_len_pid;
    PID_Controller right_len_pid;

    LQR2 lqr2;
    LQR_Yaw lqr_yaw;
    ForceCtrl force;

public:
    LPFilter2<float> roll_lpf;
    Cmd &cmd;

private:
    Robot_Class &robot;
};

#endif