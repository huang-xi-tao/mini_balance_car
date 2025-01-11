#pragma once
#ifndef PID_HPP__
#define PID_HPP__
#include <stdint.h>

class PID_Controller
{
public:
    PID_Controller() : kp(0),
                       ki(0), kd(0), err_int(0), output(0), max_int(0), max_out(0) {}
    // PI
    float update(const float target, const float input, const float dt = 0);
    // PID
    float update(const float target, const float d_target, const float input, const float d_input,
                 const float dt = 0);

    // PID with thresh error
    float update(float target, float d_target, float input, float d_input, float int_thresh_err, float dt);

    // PID with bound value
    float update(const float target, const float d_target, const float input, const float d_input,
                 const float max_int, const float min_int,
                 const float max_out, const float min_out,
                 const float dt = 0);
    // PD controller
    float updatePD(const float qdesire, const float dqdesire,
                   const float qest, const float dqest);
    /**
     * @brief: 积分项清零
     * @author: Dandelion
     * @Date: 2024-05-31 15:37:36
     * @return {*}
     */                   
    void reset(void)
    {
        this->err_int = 0;
    }

    float kp;
    float ki;
    float kd;

    float err_int;
    float output;

    float max_int;
    float max_out;
};
#endif