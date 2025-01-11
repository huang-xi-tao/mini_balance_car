/*
 * @Author: DESKTOP-KB7I0NO\admin as56296971@gmail.com
 * @Date: 2023-12-05 21:58:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-06-09 15:21:09
 * @FilePath: \my_controller\simulation\A1_def.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef ROBOT_DEF_HPP
#define ROBOT_DEF_HPP
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/accelerometer.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/mouse.h>
#include <webots/joystick.h>
#include <webots/gyro.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <unistd.h>
#include "MathFunc.hpp"

#define USE_RVIZ 1
// #define USE_GNU_PLOT 0
#define TIME_STEP 5

#define ROBOT_NAME "robot" // position of the robot node with respect to root node children
#define ACCELEROMETER "accl"
#define GYROSCOPE "gyro"
#define INETIA_UNIT "imu"
#define LEFT_WHEEL "LEFT_WHEEL"
#define RIGHT_WHEEL "RIGHT_WHEEL"
#define USE_CLOSECHAIN 0
#if USE_CLOSECHAIN
#define LINK_NUM 13U
const double init_position[MOTOR_NUM] = {0.0, 0.0, -M_PI / 2 - 0.9, 0.0, 0.0, 0.0, -M_PI / 2 - 0.9, 0.0};
#else
#define LINK_NUM 9U
// const double init_position[MOTOR_NUM] = {1.43, 2.87f, 0.0, 1.43, 2.87f, 0.0};
const double init_position[MOTOR_NUM] = {0.f, 0.f, 0.f, 0.f, 0.f};
#endif
const char motors_c[MOTOR_NUM][40] = {"joint_left_bar",
                                      "joint_left_wheel",
                                      "joint_right_bar",
                                      "joint_right_wheel",
                                      "tail_motor"};
const char pos_c[MOTOR_NUM][40] = {"joint_left_bar_sensor",
                                   "joint_left_wheel_sensor",
                                   "joint_right_bar_sensor",
                                   "joint_right_wheel_sensor",
                                   "tail_motor_sensor"};
const float torque_max[MOTOR_NUM] = {100, 100, 100, 100, 1000};
const float rpm_max[MOTOR_NUM] = {1000, 180, 1000, 180, 100};
const float torque_limit[MOTOR_NUM] = {100, 100, 100, 100, 1000};

#endif