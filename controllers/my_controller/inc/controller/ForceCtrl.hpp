#pragma once
#ifndef FORCE_CTRL_HPP__
#define FORCE_CTRL_HPP__
#include "Filter.hpp"
#include "Parameter.hpp"
#include "robot_likeAscento.hpp"

class ForceCtrl
{
public:
    ForceCtrl(Robot_Class &robot);

    void setup();

    void leg_psc_update(void);
    float accl_estimate(const float input);

    float accl_update(const float set_len, const float set_d_len, const float set_dd_leg_len,
                      const float fb_len, const float fb_d_len, float &leg_len_err);

    void reset(void)
    {
        accl_int = 0;
    }
    void Set_Force_D(float val)
    {
        force_d = val;
    }

    void Set_Force_P(float val)
    {
        force_p = val;
    }

    void Set_Force_I(float val)
    {
        force_i = val;
    }

public:
    float accl_out;
    float accl_set;
    float accl_set_left;
    float accl_set_right;

    float force_p_param = 100.f; // 36.f
    float force_d_param = 5.f;   // 3.6
private:
    Robot_Class &robot;

    float accl_kp;
    float accl_ki;
    float accl_int;

    float force_p = 0;
    float force_d = 0;
    float force_i = 0;

public:
    float accl;
    float accl_raw;

private:
    LPFilter2<float> accl_lpf;
};

#endif