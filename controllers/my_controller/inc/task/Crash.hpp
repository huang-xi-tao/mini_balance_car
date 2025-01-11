#pragma once
#include "robot_likeAscento.hpp"
#include "Controller.hpp"
#include "Planner.hpp"
#include "Init.hpp"

class Crash_Class
{

public:
    Crash_Class(Robot_Class &robot, Controller_Class &ctrl, Planner_Class &planner) : robot(robot), ctrl(ctrl), planner(planner)
    {
        this->len_ramp.val = 0;
    }
    bool Crash_checkout(float dt);

private:
    Robot_Class &robot;
    Controller_Class &ctrl;
    Planner_Class &planner;

private:
private:
    Ramp<float> len_ramp;
};