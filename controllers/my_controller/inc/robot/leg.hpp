#pragma once
#ifndef LEG_HPP__
#define LEG_HPP__
#include "Joint_motor.hpp"
#include "MathFunc.hpp"
#include "Filter.hpp"
#include "Quaternion.hpp"
#include "robot_status.hpp"
#include "Parameter.hpp"
#include "param_wheelBipedRobot.hpp"
class Leg2
{
public:
    Leg2(const uint8_t config, WB6_Parameter *param); // config 0 is left 1 is right
    ~Leg2(){};
    void Update_Leg(const float pitch, const float d_pitch, const float *q4, const float *dq4, float dt);
    // void Force_Arm_Estimate(Quaternion q, float leg_angle, float vel_forward, float vel_yaw, float roll,
    //                         bool offground);

public:
    float _q3;
    float _q1; // kinematic various
    float _q2;
    float _q4;

    float _dq1;
    float _dq2;
    float _dq3;
    float _dq4;

    float wheel_pos;
    float wheel_vel;
    float _wheelPosNow = 1.0f;
    float _sideSign; // -1 refer to right leg ,1 refer to left leg
private:
    float _legRoll;
    float _legPitch;
    float _dlegRoll;
    float _dlegPitch;
    float _initPos[4];

    float _tau3K = 1;
    const InitMotorPosition _initMotorPos;

public:
    bool ok;
    uint8_t config;
    float knee_angle;
    float sin_knee;
    float sin_knee_psc;
    float angle;
    float d_angle;
    float tilt;
    float d_tilt;
    float round_tilt;
    float cos_tilt;
    int tilt_rev;
    float len;
    float d_len;
    float dd_len;

    float force_arm;

public: // will be abuse
    // Joint_motor hip; // incomplete class
    Joint_motor knee;
    Joint_motor wheel;

    Circular_Differentiator<float, 8> diff1;
    Circular_Differentiator<float, 8> diff2;

    LPFilter2<float> diff_lpf;
    LPFilter2<float> d_tilt_lpf;

    void Start(void);

private:
    WB6_Parameter *param_ptr;
    void calLegLength(void);
};
#endif