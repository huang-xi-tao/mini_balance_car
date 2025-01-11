#pragma once
#include "robot_likeAscento.hpp"
#include "LQR_Param.hpp"
#include "Filter.hpp"
#include "ctrl_output.hpp"

class LQR2
{ // Should be more abstract
public:
    LQR2(Robot_Class &robot);

    void update(const float dt, float &set_pitch, float &set_d_pitch, float &set_tilt, float &set_d_tilt,
                const float set_forward_val,
                Ctrl_Output_Class &left_out, Ctrl_Output_Class &right_out,
                float &pos_forward_err);
    void reset(void)
    {
        headAng_int_wheel_L = headAng_int_wheel_R = 0;
    }
    void Clear_LQR(void);
    bool lqr_start_flag;
    uint8_t run_status = 0;
    float pos_cmd_err = 0;

private:
    Robot_Class &robot;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K11;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K12;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K13;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K14;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K15;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K16;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K17;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K21;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K22;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K23;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K24;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K25;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K26;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K27;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K11A;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K12A;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K13A;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K21A;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K22A;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K23A;

    float headAng_int_wheel_L;
    float headAng_int_wheel_R;
    float inc_wheel_L;
    float inc_wheel_R;
    float max_wheel_int;
    float balance_output;
    float forward_err;
    float vel_output;

    float cross_int_L;
    float cross_int_R;

    LPFilter2<float> tilt_lpf;
    float d_tilt_err;

    float head_ff(float set_pitch, const float accl[3]);

    void update_gnd(const float dt, const Leg2 *leg,
                    float &wheel_int,
                    float set_pitch, float set_d_pitch,
                    float set_tilt, float set_d_tilt,
                    float set_forward_val,
                    Ctrl_Output_Class &output,
                    float pos_forward_err);

private:
    float lqr_start_time = 0;
    float lqr_set_tilt = 0;
};
