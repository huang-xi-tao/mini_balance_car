#pragma once

#include "robot_likeAscento.hpp"
#include "LQR_Param.hpp"
#include "Filter.hpp"

class LQR_Yaw
{
public:
    LQR_Yaw(Robot_Class &robot);

    void init(void)
    {
        this->reset();
    }

    void update(const float dt,
                float &pos_yaw_err,
                Ctrl_Output_Class &left_out, Ctrl_Output_Class &right_out,
                float &set_yaw, bool init = true);

    void reset(void)
    {
    }

private:
    void update_yaw(const float dt, const float d_yaw_err, Ctrl_Output_Class &left_out, Ctrl_Output_Class &right_out,
                    float &pos_yaw_err, float &set_yaw, const bool init);

private:
    Robot_Class &robot;

    Linear_Interpolator<float, PARAM_LQR_LEVEL> K11;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K12;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K13;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K14;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K15;

    Linear_Interpolator<float, PARAM_LQR_LEVEL> K21;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K22;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K23;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K24;
    Linear_Interpolator<float, PARAM_LQR_LEVEL> K25;

    float yaw_int_L;
    float yaw_int_R;

    float yaw_target_remain_time;
    uint8_t yaw_push_status = false;
    float yaw_push_cnt = 0;

    float yaw_push_recover_cnt;
    LPFilter2<float> vel_yaw_lpf;
    float d_yaw_err_last;
    float yaw_err_test;
    float yaw_err_last_test;
    float d_yaw_err_test;
    float d_yaw_err_last_test;
};
