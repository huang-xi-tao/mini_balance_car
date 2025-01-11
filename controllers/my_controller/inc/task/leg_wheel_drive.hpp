#pragma once
#include "robot_likeAscento.hpp"
#include "Controller.hpp"
#include "Planner.hpp"
#include "Init.hpp"

class Leg_Wheel_Drive_Class
{

public:
    Leg_Wheel_Drive_Class(Robot_Class &robot, Controller_Class &ctrl, Planner_Class &planner, WB6_Parameter &param, Init_Handle &initHandle, bool init = false) : robot(robot), ctrl(ctrl), planner(planner), param(param), initHandle(initHandle)
    {
        this->len_ramp.val = 0;
    }

    void Package_Init();
    void Package_Run(const float dt);
    void Set_Init_Status(bool init)
    {
    }
    // void Set_Init_Len(float init_left_len, float init_right_len);

private:
    Robot_Class &robot;
    Controller_Class &ctrl;
    Planner_Class &planner;
    WB6_Parameter &param;

    Init_Handle &initHandle;

private:
    void knee_ctrl(float dt);
    void wheel_ctrl(float dt);
    void output_mix(float dt);

private:
    Ramp<float> len_ramp;
};