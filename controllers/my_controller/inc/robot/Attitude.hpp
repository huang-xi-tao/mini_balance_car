#pragma once
#ifndef ATTITUDE_CLASS_HPP
#define ATTITUDE_CLASS_HPP
#include <stdint.h>
#include <string.h>
#include "Quaternion.hpp"
#include "Parameter.hpp"
#include "leg.hpp"
#include "MathFunc.hpp"

class Attitude_Class
{
public:
    Attitude_Class(WB6_Parameter &param);
    Attitude_Class(){};

public:
    void getBodyPosture(float stamp);
    // float getRobotHeight();
    // void calcCrossLink(float stamp);
    // void calcParallelLink(float stamp);

public:
    float _body2imu[6]; // set IMU transform xyz rqy

    float _roll;
    float _pitch;
    float _yaw;
    float _droll;
    float _dpitch;
    float _dyaw;
    float _xaccl;
    float _yaccl;
    float _zaccl;
    float _xvel;
    float _yvel;
    float _zvel;
    float _qMotor[MOTOR_NUM];
    float _dqMotor[MOTOR_NUM];
    float _ddqMotor[MOTOR_NUM];
    float _tauMotor[MOTOR_NUM];
    // float _qfloatingEst[FLOATING_BASE];
    // float _dqfloatingEst[FLOATING_BASE];
    float _qbodyEst[MOTOR_NUM] = {0};
    float _dqbodyEst[MOTOR_NUM] = {0};
    float _ddqbodyEst[MOTOR_NUM] = {0};
    float _tauEst[MOTOR_NUM];
    float _tau3K[2] = {0};
    Quaternion _quaternion;
    // const KinematicParam _kinParam;
    // const DynamicParam _dynParam;
    // const InitMotorPosition _initMotorPos;

    float _legLen[2] = {0};
    float _legTilt[2] = {0};
    float _dlegLen[2] = {0};
    float _dlegTilt[2] = {0};

    float accl[3] = {0};
    float rotation[3] = {0};

public:
    float head_pitch;
    float d_head_pitch;
    float tilt;

    float d_tilt;

    float leg_angle;
    float leg_len;

    float height = 0;
    float vel_forward = 0;
    float vel_yaw = 0;
    float round_tilt = 0;
    float cos_tilt = 0;
    int tilt_rev = 0;
    float ground_tilt = 0;

    float roll;

    float roll_curve;

    float pitch_last;
    float d_pitch_last = 0;
    float tilt_angle_update(const float input, float &round_ang, float rev)
    {
        if (input > M_PI && round_ang < -M_PI)
            rev--;
        else if (input < -M_PI && round_ang > M_PI)
            rev++;

        round_ang = input;
        return (round_ang + tilt_rev * 2 * M_PI);
    }

    // bool     init;

    float yaw;
    float d_yaw;
    int yaw_rev;
    uint32_t update_cnt;

public:
    void pitch_estimate(void); // TODO Modify
    void roll_estimate(float legL_len, float legR_len, float wheel_distance);
    void wheel_estimate(const float wheel_radius, const float dt, Leg2 &legL, Leg2 &legR);
    void tilt_estimate(WB6_Parameter &param, Leg2 &legL, Leg2 &legR);
    void update_heading(void);
    void attitude_update(float stamp);

    float pos_forward;
    void Pass_Left_Leg_Ptr(Leg2 *leg)
    {
        legL_ptr = leg;
    }
    void Pass_Right_Leg_Ptr(Leg2 *leg)
    {
        legR_ptr = leg;
    }

private:
    Circular_Differentiator<float, 8> diff0;
    Circular_Differentiator<float, 8> diff1;
    Circular_Differentiator<float, 8> diff2;
    Circular_Differentiator<float, 8> diff3;
    Circular_Differentiator<float, 8> diff4;
    Circular_Differentiator<float, 8> diff5;
    Circular_Differentiator<float, 8> diff6;
    Circular_Differentiator<float, 8> diff7;

    Circular_Intergrator<float, 8> inter0;
    Circular_Intergrator<float, 8> inter1;
    Circular_Intergrator<float, 8> inter2;
    Circular_Differentiator<float, 8> dgamaL;
    Circular_Differentiator<float, 8> dgamaR;

    Leg2 *legL_ptr;
    Leg2 *legR_ptr;
    WB6_Parameter *param_ptr;
};

#endif