#include "ForceCtrl.hpp"

ForceCtrl::ForceCtrl(Robot_Class &robot) : robot(robot),
                                           accl_lpf(1000.f, 40.f)
{
    accl_set = accl_out = 0;
    accl = accl_raw = 0;
}

void ForceCtrl::setup()
{
    this->accl_kp = 0;
    this->accl_ki = 0;
    this->accl_int = 0;
}

float ForceCtrl::accl_estimate(const float input)
{
    this->accl_raw = input;
    this->accl = accl_lpf.update(input);

    return this->accl;
}
/**
 * @brief: 高度环的PID在这里
 * @author: Dandelion
 * @Date: 2024-05-31 17:10:00
 * @return {*}
 */
float ForceCtrl::accl_update(const float set_len, const float set_d_len, const float set_dd_leg_len,
                             const float fb_len, const float fb_d_len, float &leg_len_err)
{
    float error = set_len - fb_len,
          d_error = set_d_len - fb_d_len;

    float force_f = set_dd_leg_len; // 0, in
    leg_len_err = error;

    float result = force_f + force_p_param * error + force_d_param * d_error;
    // if (!robot.flag_ptr->offground)
    // {
    const float min_accl = -1;
    if (result < min_accl)
        result = min_accl;
    // }
    return result;
}
