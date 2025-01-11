#include "Task.hpp"
#include <math.h>
#include <stdio.h>
using namespace Command;

void Task_Class::Run_Slow_Init()
{
}
void Task_Class::Run_Medium_Init()
{
}
void Task_Class::Run_Fast_Init(void)
{
    initHandle.Package_Init();
    initHandle.Set_Package_Status(initHandle.INIT_RESET);
    planner.plan_start();
}

void Task_Class::Run_Slow(float dt)
{
}

void Task_Class::Run_Medium(float dt)
{
    switch (medium_status)
    {
    case MEDIUM_INIT_STATUS:
        if (this->fast_status == FAST_INIT_MODAL)
        {
            medium_status = MEDIUM_INIT_MODAL;
        }
        break;
    case MEDIUM_INIT_MODAL:
        planner.Clear_Pos_Err();
        if (this->fast_status == FAST_LEG_WHEEL_MODAL)
        {
            medium_status = MEDIUM_LEG_WHEEL_MODAL;
        }
        break;
    case MEDIUM_LEG_WHEEL_MODAL:
        planner.plan_update(dt);
        break;
    }
    planner.wdg_check_update(dt);
    // if (crash_handle.Crash_checkout(dt))
    // {
    //     this->fast_status = Task_Class::KILL;
    //     if (robot.cmd.crash_restar)
    //     {
    //         // robot.cmd.kill = false;
    //         robot.cmd.crash_restar = false;
    //         this->fast_status = Task_Class::FAST_INIT_STATUS;
    //     }
    // }
    if (this->planner.reach_ground)
    {
        this->planner.reach_ground = false;
        this->set_kill(true);
        this->planner.kill_flag = true;
    }
    if (robot.wdg.Error())
    {
        ctrl.lqr2.reset();
        ctrl.lqr_yaw.reset();
        planner.wheel_cmd_reset();
        // planner.jumpHandle.reset();
        ctrl.drive_pid.ki = 0.0f;
        ctrl.drive_pid.max_int = 0.f;
        initHandle.reset_init_val();
        this->fast_status = Task_Class::KILL;
        robot.cmd.transform = Command::TRANSFORM_IDLE;
        planner.jumpHandle.tilt_cmd_reset();
    }
    else if (this->planner.kill_flag && !initHandle.get_init_val() && this->start_reset == true)
    {
        this->fast_status = Task_Class::FAST_INIT_STATUS;
        this->planner.kill_flag = false;
    }
    // else if(initHandle.get_init_val())
    // this->fast_status = Task_Class::FAST_LEG_WHEEL_MODAL;
}

void Task_Class::Run_Fast(float dt)
{
    if (dt > 0.01f)
        dt = 0.01f;
    robot.Robot_Update(dt);

    switch (this->fast_status)
    {
    case Task_Class::FAST_INIT_STATUS:
        fast_time += dt;
        if (!this->cmd.kill && fast_time > 0.1f)
        {
            this->Run_Fast_Init();
            this->fast_status = Task_Class::FAST_INIT_MODAL;
        }

        break;

    case Task_Class::FAST_INIT_MODAL:
        initHandle.Package_Run(dt);

        if (initHandle.get_init_val())
        {
            ctrl.transform(Robot_Class::CAR);
            this->fast_status = Task_Class::FAST_LEG_WHEEL_MODAL;
        }
        break;

    case Task_Class::FAST_LEG_WHEEL_MODAL:
        leg_wheel_drive_handle.Package_Run(dt);
        if (robot.cmd.crash_count > 5000)
        {
            this->fast_status = Task_Class::FAST_INIT_STATUS;
        }
        break;

    case Task_Class::ERROR:

        break;

    case Task_Class::KILL:
        ctrl.kill_robot();
        if (!cmd.kill)
        {
            this->fast_status = Task_Class::FAST_INIT_MODAL;
        }
        break;

    default:
        break;
    }
    ctrl.output();
}
