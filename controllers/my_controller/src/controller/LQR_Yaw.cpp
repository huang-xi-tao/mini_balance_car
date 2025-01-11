#include "LQR_Yaw.hpp"
#include "stdio.h"

LQR_Yaw::LQR_Yaw(Robot_Class &robot) : robot(robot),
                                       K11(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K11),
                                       K12(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K12),
                                       K13(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K13),
                                       K14(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K14),
                                       K15(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K15),

                                       K21(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K21),
                                       K22(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K22),
                                       K23(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K23),
                                       K24(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K24),
                                       K25(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.yaw.K25)
{
    this->init();
    vel_yaw_lpf.init(1000, 100);
    yaw_err_last_test = 0;
    d_yaw_err_last_test = 0;
    yaw_target_remain_time = 0;
    this->yaw_int_L = 0;
    this->yaw_int_R = 0;
}

void LQR_Yaw::update(const float dt,
                     float &pos_yaw_err,
                     Ctrl_Output_Class &left_out, Ctrl_Output_Class &right_out,
                     float &set_yaw, bool init)
{
    if (robot.cmd.yaw.d_val >= -0.01 &&
        robot.cmd.yaw.d_val <= 0.01 &&
        robot.cmd.forward.d_val >= -0.01 &&
        robot.cmd.forward.d_val <= 0.01) // RC deadzone
    {
        if (robot.mode == Robot_Class::CAR)
        {
            robot.cmd.yaw.d_val = 0;
        }
    }

    float d_yaw_err = vel_yaw_lpf.update(robot.cmd.yaw.d_val - robot.posture_ptr->vel_yaw);
    // float yaw_err_del =  robot.cmd.yaw.d_val - robot.posture_ptr->vel_yaw;
    // float d_yaw_err = yaw_err_del * 0.2f + 0.8f * d_yaw_err_last;
    // d_yaw_err_last =  d_yaw_err;

    float yaw_err = 0;

    if (init)
    {
        if (mabs(robot.cmd.yaw.d_val) >= 0.05f) // control the yaw angle
        {
            yaw_target_remain_time = 0;
            set_yaw += robot.cmd.yaw.d_val * dt; // record the target yaw output
        }
        else if (robot.cmd.yaw.d_val == 0 && robot.cmd.forward.d_val == 0 && (mabs(robot.posture_ptr->vel_yaw) <= 0.01f) && (mabs(robot.posture_ptr->vel_forward) <= 0.1f)) // to avoid yaw sliding, record the yaw angle in every 2 seconds when the robot is stopped
        {
            yaw_target_remain_time += dt;
            if (yaw_target_remain_time > 2.f)
            {
                yaw_target_remain_time = 0;
                set_yaw = robot.posture_ptr->yaw;
                set_yaw = 0;
            }
        }
        yaw_err = set_yaw - robot.posture_ptr->yaw;

        if (yaw_err > 3.1415926f)
        {
            pos_yaw_err = yaw_err - 2 * 3.1415926f;
        }
        else if (yaw_err < -3.1415926f)
        {
            pos_yaw_err = yaw_err + 2 * 3.1415926f;
        }
        else
        {
            pos_yaw_err = yaw_err;
        }
        // limit the error
        if (pos_yaw_err > M_PI / 6.f)
        {
            pos_yaw_err = M_PI / 6.f;
            set_yaw = robot.posture_ptr->yaw - pos_yaw_err;
        }
        else if (pos_yaw_err < -M_PI / 6.f)
        {
            pos_yaw_err = -M_PI / 6.f;
            set_yaw = robot.posture_ptr->yaw + pos_yaw_err;
        }
    }
    else
    {
        set_yaw = robot.posture_ptr->yaw;
    }
    update_yaw(dt, d_yaw_err, left_out, right_out, pos_yaw_err, set_yaw, init);
}

void LQR_Yaw::update_yaw(const float dt, const float d_yaw_err, Ctrl_Output_Class &left_out, Ctrl_Output_Class &right_out,
                         float &pos_yaw_err, float &set_yaw, const bool init)
{
    if (!init)
    {
        left_out.yaw_out = right_out.yaw_out = 0;
        return;
    }

    float yaw_err_del = d_yaw_err;

    this->yaw_int_L += /*  pos_yaw_err*/ yaw_err_del * 0.01f; // * 0.001f;
    this->yaw_int_R += /*  pos_yaw_err*/ yaw_err_del * 0.01f; // * 0.001f;

    bound(yaw_int_L, 3.f);
    bound(yaw_int_R, 3.f);

    left_out.yaw_out = K11.getVal(robot.legL_ptr->len) * pos_yaw_err * 2.0f + K12.getVal(robot.legL_ptr->len) * yaw_err_del * 3.5f;
    right_out.yaw_out = K11.getVal(robot.legR_ptr->len) * pos_yaw_err * 2.0f + K12.getVal(robot.legR_ptr->len) * yaw_err_del * 3.5f;
    left_out.yaw_out += this->yaw_int_L;
    right_out.yaw_out += this->yaw_int_R;
}
