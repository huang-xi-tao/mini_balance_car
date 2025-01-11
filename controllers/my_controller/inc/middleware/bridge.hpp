#pragma once
#ifndef BRIDGE_HPP__
#define BRIDGE_HPP__
#include "stdint.h"
#include "MathConst.h"
#define USE_QUATERNION 1

class Down2Up_Class
{
public:
    float gyro[3] = {0};
    float accl[3] = {0};
#if USE_QUATERNION
    float q[4] = {0};
#endif
    float torque_fbk[MOTOR_NUM] = {0};
    float velocity_fbk[MOTOR_NUM] = {0};
    float angle_fbk[MOTOR_NUM] = {0};
    float battery_vol = {0};
    float battery_curr = {0};
    float battery_capacity = {0};
    float imu_transform_to_base[6] = {0};
};

class Up2Down_Class
{
public:
    bool sensor_mode; // 0:General sensing mode 1:Enhanced sensing mode (reserved for other sensor values)
    bool stand_up;    // 0:stand up 1:sit down
    bool brake;       // 0:nothing 1:Operation control output shut down
    uint8_t sdk_mode;
    uint8_t modal;        // 0:Labor-saving mode 1:normal mode2:Wheel mode 3:Hybrid mode
    uint8_t operate_mode; // 0:Normal low speed mode 1:Anti-interference 2.mode Load mode 3:normal high speed mode

    float cmd_forward_vel; //-10m/s~10m/s
    float cmd_yaw_vel;     //-10 rad/s ~ 10rad/s
    float cmd_lateral_vel; //-1.5m/s ~ 1.5m/s
    float roll_angle;      //-10 rad ~ 10 rad
    float height;          // 0 ~ 0.7 m
    float imu_bias_y;      //-0.5 rad ~ 0.5 rad
    float imu_bias_x;      //-0.5 rad ~ 0.5 rad
    float torque[8];       //-40 Nm ~ 40 Nm

    uint8_t robot_mode; // 0~8

    float torque_set[MOTOR_NUM] = {0};
    float angle_set[MOTOR_NUM] = {0};
    float forward_vel;   //-10 ~ 10 m/s
    float yaw_vel;       //-20 rad/s ~ 20 rad/s
    float tilt;          //-3.14159 ~ 3.14159 rad
    float d_tilt;        //-20 rad/s ~ 20 rad/s
    float pitch;         //-3.14159 ~ 3.14159 rad
    float d_pitch;       //-20 rad/s ~ 20 rad/s
    float legL_len;      // 0m~ 0.5m
    float legR_len;      // 9m~0.5m
    float quat[4];       // 0~1
    float omega_body[3]; //-50rad/s ~ 50 rad/s
    float acc_body[3];   //-100m/s^2~100m/s^2
    uint32_t error;      // robot_error
};

#endif