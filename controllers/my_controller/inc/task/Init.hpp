#pragma once

#include <stdint.h>
#include "Ramp.hpp"
#include "robot_likeAscento.hpp"
#include "Controller.hpp"

class Init_Handle
{

public:
    // 功能包初始化
    void Package_Init()
    {
        started = false;
        init = true;
    }
    // 功能包的主调度函数。注意，Package Run视乎运行频率，可以有多个，比如多态，或者Package_Run_2,Package_Run_3,等
    // 但不宜过多
    void Package_Run(float dt);

    typedef enum
    {
        INIT_UPDATE = 0,
        INIT_RESET, // reset then init status

    } Package_Status_t;

    typedef enum
    {
        UPDATE_START = 0,
        UPDATE_LEN_RESET,
        UPDATE_ANG_RESET,
        UPDATE_ANG_RESET_BACK,
        UPDATE_STATUS_RESET,
        UPDATE_FIN,
        UPDATE_CHECK,
    } Init_Step_t;

    // 功能包输入接口示例
    bool Get_Reset_Mode()
    {
        return reset_mode;
    }

    bool get_init_val()
    {
        return init;
    }
    void reset_init_val()
    {
        init = false;
    }

    Init_Step_t Get_Init_Step(void) // Get the init_step
    {
        return init_step;
    }

    void Set_Init_Step(Init_Step_t val) // Get the init_step
    {
        init_step = val;
    }

    // 功能包输出接口示例
    void Set_Package_Status(Package_Status_t val)
    {
        package_status = val;
    }

private:
    // 不对外暴露变量，只暴露get set 接口
    Init_Step_t init_step = UPDATE_START;
    Package_Status_t package_status = INIT_UPDATE;

    bool started;
    bool init;
    uint8_t reset_trial;
    uint8_t reset_mode;
    float reset_time;

public:
    Init_Handle(Robot_Class &robot, Controller_Class &ctrl, bool init = false) : init(init), robot(robot), ctrl(ctrl), reset_trial(0)
    {
        this->started = false;
        this->init_ang = 0;
    }

    // function
private:
    void Start();
    void Update(const float dt);

    void start(const float init_d_len = 0.4f, const float init_d_ang = M_PI / 2);
    void reset(const float init_d_len = 0.8f, const float init_d_ang = 2 * M_PI);
    void update(const float dt);
    bool stable(void)
    {
        return !reset_trial;
    }

    float leg_angle(const float dest)
    {
        return init ? dest : ang_ramp.getResult(init_ang, dest);
    }

    float d_leg_angle(const float dest)
    {
        return init ? dest : ang_ramp.getDResult(init_ang, dest);
    }

    float left_leg_len(void)
    {
        return init ? robot.param_ptr->core.LEG_LEN_RETRACT : len_ramp.getResult(init_left_len, robot.param_ptr->core.LEG_LEN_RETRACT);
    }

    float right_leg_len(void)
    {
        return init ? robot.param_ptr->core.LEG_LEN_RETRACT : len_ramp.getResult(init_right_len, robot.param_ptr->core.LEG_LEN_RETRACT);
    }
    Ramp<float> len_ramp;
    Ramp<float> ang_ramp;

    // method
private:
    float right_leg_angle(const float start, const float dest)
    {
        return init ? dest : ang_ramp.getResult(start, dest);
    }

    float left_leg_angle(const float start, const float dest)
    {
        return init ? dest : ang_ramp.getResult(start, dest);
    }

    float init_left_len;
    float init_right_len;

    Ramp<float> l_len_ramp;
    Ramp<float> l_ang_ramp;

    Ramp<float> r_len_ramp;
    Ramp<float> r_ang_ramp;

private:
    Robot_Class &robot;
    Controller_Class &ctrl;

    float init_ang;
    float init_d_len;
    float init_d_ang;

    uint8_t l_step = UPDATE_START;
    uint8_t r_step = UPDATE_START;

public:
    float _initAngle[MOTOR_NUM] = {0};
    float _initLegLength[2];
    float _initLegTilt[2];

    Ramp<float> _legLenRamp;
    Ramp<float> _legTiltRamp;
    Ramp<float> _WheelRamp;
};
