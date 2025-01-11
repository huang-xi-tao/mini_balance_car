#include "LQR2.hpp"
LQR2::LQR2(Robot_Class &robot) : robot(robot),
                                 K11(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K11),
                                 K12(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K12),
                                 K13(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K13),
                                 K14(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K14),
                                 K15(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K15),
                                 K16(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K16),
                                 K17(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K17),

                                 K21(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K21),
                                 K22(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K22),
                                 K23(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K23),
                                 K24(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K24),
                                 K25(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K25),
                                 K26(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K26),
                                 K27(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K27),

                                 K11A(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K11A),
                                 K12A(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K12A),
                                 K13A(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K13A),

                                 K21A(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K21A),
                                 K22A(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K22A),
                                 K23A(robot.param_ptr->core.LEG_LEN_STRAIGHT / PARAM_LQR_LEVEL, robot.param_ptr->core.LEG_LEN_STRAIGHT, robot.param_ptr->lqr.gnd.K23A)
{
    this->reset();
    tilt_lpf.init(1000, 50);

    this->max_wheel_int = 0;
}

void LQR2::Clear_LQR(void)
{
    headAng_int_wheel_L = 0;
    headAng_int_wheel_R = 0;
    lqr_start_flag = 0;
    lqr_start_time = 0;
}

void LQR2::update(const float dt, float &set_pitch, float &set_d_pitch, float &set_tilt, float &set_d_tilt,
                  float set_forward_val,
                  Ctrl_Output_Class &left_out, Ctrl_Output_Class &right_out,
                  float &pos_forward_err)
{
    this->update_gnd(dt, robot.legL_ptr,
                     headAng_int_wheel_L,
                     set_pitch, set_d_pitch,
                     set_tilt, set_d_tilt, set_forward_val,
                     left_out,
                     pos_forward_err);

    this->update_gnd(dt, robot.legR_ptr,
                     headAng_int_wheel_R,
                     set_pitch, set_d_pitch,
                     set_tilt, set_d_tilt, set_forward_val,
                     right_out,
                     pos_forward_err);
}

void LQR2::update_gnd(const float dt, const Leg2 *leg,
                      float &wheel_int,
                      float set_pitch, float set_d_pitch,
                      float set_tilt, float set_d_tilt,
                      float set_forward_val,
                      Ctrl_Output_Class &output,
                      float pos_forward_err)
{
    static float gnd_cnt = 0;
    static float off_cnt = 0.f;
    // static float forward_err = 0.f;
    static float d_forward_err = 0.f;
    static float pitch_psc = 1.5f, d_pitch_psc = 2.0f;
    static float tilt_psc = 1.5f, d_tilt_psc = 2.0f;
    float vel_psc = 1.1f, d_vel_psc = 1.1f;

    // status judge fordifferent lqr gain
    // 1 启动   2 静止 3加速   4 减速
    if (!lqr_start_flag)
    {
        run_status = 0;
    }

    switch (run_status)
    {
    case 0:
        vel_psc = 1.5f;
        d_vel_psc = 1.5f;
        set_forward_val = 0.f;
        lqr_start_time += dt;
        if (lqr_start_time > 2.f)
        {
            run_status = 1;
            lqr_start_flag = true;
            lqr_start_time = 0.f;
        }
        break;
    case 1:
        if (set_forward_val > 1.f)
        {
            run_status = 2;
        }
        else if (set_forward_val < -1.f)
        {
            run_status = 3;
        }
        else
        {
            set_forward_val = 0.f;
        }
        break;
    case 2: // 前进
        if (set_forward_val <= 0.5f)
        {
            run_status = 4;
        }
        break;
    case 3: // 后退
        if (set_forward_val >= -0.5f)
        {
            run_status = 4;
        }
        break;
    case 4:
        if (set_forward_val > 1.f)
        {
            run_status = 2;
            lqr_start_time = 0.f;
        }
        else if (set_forward_val < -1.f)
        {
            run_status = 3;
            lqr_start_time = 0.f;
        }
        else
        {
            if (mabs(robot.posture_ptr->vel_forward) < 0.2f || mabs(pos_cmd_err) > 3.f)
            {
                lqr_start_time += dt;
            }
            if (lqr_start_time > 1.f)
            {
                run_status = 1;
                lqr_start_time = 0.f;
            }
        }

        break;
    }

    forward_err = pos_forward_err * robot.param_ptr->core.WHEEL_RADIUS;
    d_forward_err = (set_forward_val * robot.param_ptr->core.WHEEL_RADIUS - robot.posture_ptr->vel_forward);

    // lqr_set_tilt = 0; // atan(l * cos(-robot.posture_ptr->head_pitch) / (len + l * sin(-robot.posture_ptr->head_pitch)));
    lqr_set_tilt = set_tilt;
    wheel_int += K12.getVal(leg->len) * forward_err * dt / 2.f;
    // 加入tilt前馈项
    float CoM_x_Tail = -0.5 * robot.param_ptr->core.TAIL_LEN * cosf(robot.cmd.tail_angle.val) + 0.1;
    float CoM_x = CoM_x_Tail * robot.param_ptr->core.TAIL_MASS / (robot.param_ptr->core.TAIL_MASS + robot.param_ptr->core.UPPER_MASS);
    float theta = atanf(CoM_x / (robot.posture_ptr->height + 0.3));
    switch (robot.mode)
    {
    case robot.TRANSFORM_DOWN:
        /* code */
        lqr_set_tilt = -theta;
        if (robot.tail_phase_down == 2 && robot.posture_ptr->height < robot.param_ptr->core.MIN_HEIGHT + 0.03f)
        { // 收尾阶段的时候往前倒
            lqr_set_tilt = 0.2;
        }
        break;
    case robot.TRANSFORM_UP:
    case robot.STAND:
        lqr_set_tilt = -theta;
        break;

    default:
        break;
    }
    // lqr_set_tilt += 10.f * robot.posture_ptr->tilt * dt;
    // bound(lqr_set_tilt, 0.05f, -0.05f);

    // balance_output = K14.getVal(leg->len) * (set_pitch - robot.posture_ptr->head_pitch) * pitch_psc +
    //                  K15.getVal(leg->len) * (0 - robot.posture_ptr->d_head_pitch) * d_pitch_psc;
    balance_output = K14.getVal(leg->len) * (lqr_set_tilt - robot.posture_ptr->tilt) * tilt_psc +
                     K15.getVal(leg->len) * (set_d_tilt - robot.posture_ptr->d_tilt) * d_tilt_psc;

    max_wheel_int = 5.f; //- mabs(balance_output);
    bound(wheel_int, max_wheel_int);

    // wheel_int = 0;
    vel_output = K16.getVal(leg->len) * forward_err * vel_psc +
                 K17.getVal(leg->len) * d_forward_err * d_vel_psc +
                 wheel_int;

    float min_vel_output = -(leg->wheel.max_torque * 0.8f);
    float max_vel_output = -(leg->wheel.min_torque * 0.8f);

    output.forward_out = balance_output + vel_output;
}
