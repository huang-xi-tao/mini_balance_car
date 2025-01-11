#pragma once

#include "Attitude.hpp"
#include "Power.hpp"
#include "Watchdog.hpp"
#include "Cmd.hpp"
#include "leg.hpp"
#include "Parameter.hpp"
// #include "Onboard_SDK_UART_Protocol.h"
#include "middleware.hpp"
#include "robot_status.hpp"
#include "ctrl_output.hpp"

class CANFD_R
{
public:
    bool offground; // 0,1

    uint8_t robot_mode; // 0~8
    uint8_t jump_phase; // 0~8

    float legL_load;     //-300 ~ 300 (N)
    float legR_load;     //-300 ~ 300 (N)
    float forward_vel;   //-10 ~ 10 m/s
    float yaw_vel;       //-20 rad/s ~ 20 rad/s
    float tilt;          //-3.14159 ~ 3.14159 rad
    float d_tilt;        //-20 rad/s ~ 20 rad/s
    float pitch;         //-3.14159 ~ 3.14159 rad
    float d_pitch;       //-20 rad/s ~ 20 rad/s
    float legL_len;      // 0m~ 0.5m
    float legR_len;      // 9m~0.5m
    float height;        // 0m ~ 0.5m
    float quat[4];       // 0~1
    float omega_body[3]; //-50rad/s ~ 50 rad/s
    float acc_body[3];   //-100m/s^2~100m/s^2
    uint32_t error;      // robot_error

private:
};

class CANFD_R_W
{
public:
    bool sensor_mode; // 0:General sensing mode 1:Enhanced sensing mode (reserved for other sensor values)
    bool stand_up;    // 0:stand up 1:sit down
    bool brake;       // 0:nothing 1:Operation control output shut down
    bool jump_cmd;    // 0：stop 1： start

    uint8_t jump_mode;    // 0:free jump   1:standing broad jump  2:free jump
    uint8_t jump_force;   // 0~100
    uint8_t modal;        // 0:Labor-saving mode 1:normal mode2:Wheel mode 3:Hybrid mode
    uint8_t operate_mode; // 0:Normal low speed mode 1:Anti-interference 2.mode Load mode 3:normal high speed mode

    float cmd_forward_vel; //-10m/s~10m/s
    float cmd_yaw_vel;     //-10 rad/s ~ 10rad/s
    float jump_angle;      //-1 rad ~ 1rad
    float cmd_lateral_vel; //-1.5m/s ~ 1.5m/s
    float roll_angle;      //-10 rad ~ 10 rad
    float height;          // 0 ~ 0.7 m
    float imu_bias_y;      //-0.5 rad ~ 0.5 rad
    float imu_bias_x;      //-0.5 rad ~ 0.5 rad
    float torque[8];       //-40 Nm ~ 40 Nm

private:
};
class Robot_Class
{
public:
    Robot_Class(WB6_Parameter &param, Middleware_Class &middle);

    enum flag_type_t
    {
        BALANCE = 0,
        LEFT_OFFGROUND,
        LEG_ON,
        OK, // robot can run
        OFFGROUND,
        OFFGROUND_STABLE,
        PITCH_LIMIT,
        RIGHT_OFFGROUND,
        SDK_DEBUG,
        OFFGROUND_TEST,
    };

    enum leg_mark_t
    {
        LEFT = 0,
        RIGHT,
    };

    void cmd_update();

    void Robot_Update(float dt);

    void Set_Robot_Flag(flag_type_t flag, bool status);

    void Set_Output(Ctrl_Output_Class left_out, Ctrl_Output_Class right_out);

    void Clear_Robot_Flag(void);

    void Tail_Down_FirstTime();
    void Tail_Down(float angle_set);
    void Tail_Up();

    const Status_Flag_Class *flag_ptr;
    const Leg2 *legL_ptr;
    const Leg2 *legR_ptr;
    const Power_Dummy *power_ptr;
    const Attitude_Class *posture_ptr;
    const WB6_Parameter *param_ptr;
    Watchdog wdg;

private:
    Status_Flag_Class flag;
    Leg2 legL;
    Leg2 legR;
    Joint_motor Tail_motor;
    WB6_Parameter &param;
    Attitude_Class posture;
    Power_Dummy power;

    Middleware_Class *middle_ptr = NULL;

public:
    enum robot_mode_t
    {
        CAR = 0,
        STAND = 1,
        TRANSFORM_UP = 2,
        TRANSFORM_DOWN = 3,
        REVERSE_TRANSFORM_UP = 4,
        CLASH_MODE = 5,
        TRANSFORM_REST = 6,
    };

    robot_mode_t mode;

    enum robot_condition_t
    {
        INITIALIZE = 0,
        NORMAL = 1,
        CRASH = 3,
        KILL = 4,
        TEST_ROBOT = 5,
        WBC_TEST = 6,

    };

    robot_condition_t condition;

public:
    double count_time;
    double count_time_ref;
    double run_time;
    uint8_t dance_step = 0;
    uint8_t tail_phase_down = 0;
    uint8_t tail_phase_up = 0;
    float speed_psc = 0.5f;

public:
    Cmd cmd; // the command set that in currently in control

    uint8_t cmd_num;

    /**
     * @brief check hardware error and warning
     */
    bool hardware_check(void)
    {
        bool result = (wdg.check((FALLOVER_MASK), Watchdog::CRITICAL));

        if (result)
            this->flag.ok = false;
        return !flag.ok;
    }

    /**
     *  @brief decide which command set is in control
     */
    void cmd_switch(void);

private:
    float stamp = 0;
    LPFilter2<float> _Fextlpf[12];
};
