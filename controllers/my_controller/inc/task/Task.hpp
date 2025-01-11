#pragma once

#include "robot_likeAscento.hpp"
#include "Planner.hpp"
#include "Controller.hpp"
#include "Init.hpp"
#include "Jump.hpp"
#include "Transform.hpp"
#include "LQR2.hpp"
#include "ForceCtrl.hpp"
#include "leg_wheel_drive.hpp"
#include "Crash.hpp"
class Task_Class
{
public:
    Task_Class(Robot_Class &robot, WB6_Parameter &param, Controller_Class &ctrl, Planner_Class &planner,
               const bool init = false, const Robot_Class::robot_mode_t mode = Robot_Class::CAR,
               const Robot_Class::robot_condition_t condition = Robot_Class::INITIALIZE) : robot(robot),
                                                                                           param(param),
                                                                                           ctrl(ctrl),
                                                                                           planner(planner),
                                                                                           cmd(robot.cmd),
                                                                                           wdg(robot.wdg),
                                                                                           initHandle(robot, ctrl, init),
                                                                                           leg_wheel_drive_handle(robot, ctrl, planner, param, initHandle),
                                                                                           crash_handle(robot, ctrl, planner)
    {
        robot.count_time = 0;
        robot.count_time_ref = 0;
        robot.run_time = 0;
        robot.mode = mode;
        robot.condition = condition;
        fast_status = FAST_INIT_STATUS;
        medium_status = MEDIUM_INIT_STATUS;
    }

    typedef enum
    {
        FAST_INIT_STATUS = 0,
        FAST_INIT_MODAL,       // Handle initialization
        FAST_SAFE_MODAL,       // Handle safety case
        FAST_LEG_WHEEL_MODAL,  // handle leg_wheel_modal
        FAST_HYBRID_LEG_MODAL, // handle hybrid modal
        FAST_LOW_POWER_MODAL,  // handle low power mode
        FAST_LEG_MODAL,        // handle leg mode
        ERROR,
        KILL

    } Run_Fast_Package_Status_t;

    typedef enum
    {
        MEDIUM_INIT_STATUS = 0, // 初始化，只跑一遍
        MEDIUM_INIT_MODAL,
        MEDIUM_SAFE_MODAL,
        MEDIUM_LEG_WHEEL_MODAL,
        MEDIUM_HYBRID_MODAL,
        MEDIUM_LOW_POWER_MODAL,
        MEDIUM_LEG_MODAL,

    } Run_Medium_Package_Status_t;

    void Run_Fast_Init();

    void Run_Fast(float dt); // for fast function package

    void Run_Medium_Init(); // for medium function package

    void Run_Medium(float dt); // for medium function package

    void Run_Slow_Init(); // for slow function package

    void Run_Slow(float dt); // for slow function package

private:
    Run_Fast_Package_Status_t fast_status;
    float fast_time = 0;

private:              // robot_task interface
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

private:
    Robot_Class &robot; // pointer / reference as the one way arrow

    Controller_Class &ctrl;
    Planner_Class &planner;

public:
    // CANFD write messege
    void set_robot_vel(float cmd_forward_vel, float cmd_yaw_vel);
    void set_robot_pitch(float cmd_pitch_input, Command::input_mode_t cmd_mode = Command::INPUT_POS);
    void set_modal(uint8_t modal);
    void set_stand(bool stand_up); // Stand the robot from car mode,true : stand up,  false: sit down
    void set_operate_mode(uint8_t operate_mode);
    void set_brake(bool brake);
    void set_cmd_lateral_vel(float cmd_lateral_vel);
    void set_roll_angle(float roll_angle);
    void set_height(float height, Command::input_mode_t cmd_mode = Command::INPUT_POS);
    void set_sensor_mode(uint8_t sensor_mode);
    void set_imu_bias(float bias_x, float bias_y);
    void set_kill(bool kill); // kill the motion control
    void set_cmd_reset(void); // clear cmd to default

    // CANFD get messege

    const WB6_Parameter *get_robot_param();

    void get_robot_vel(float &cmd_forward_vel, float &cmd_yaw_vel);
    void get_modal(uint8_t &modal);
    void get_stand(bool &stand_up);
    void get_operate_mode(uint8_t &operate_mode);
    void get_brake(bool &brake);
    void get_cmd_lateral_vel(float &cmd_lateral_vel);
    void get_roll_angle(float &roll_angle);
    void get_height(float &height);
    void get_sensor_mode(uint8_t &sensor_mode);
    void get_imu_bias(float &bias_x, float &bias_y);
    void get_motor_torque(float &torque, uint8_t id);

    void get_robot_mode(uint8_t &mode);
    void get_vel(float &forward_vel, float &yaw_vel);
    void get_tilt(float &l_tilt, float &l_d_tilt, float &r_tilt, float &r_d_tilt);
    void get_pitch(float &pitch, float &d_pitch);
    void get_leg_len(float &left_len, float &right_len);
    void get_quat(float *quat); // Get quaternion, w,x,y,z;
    void get_omega_body(float *omega_body);
    void get_acc_body(float *acc_body);
    void get_error(uint32_t *error);

    void canfd_read_update(void);
    void canfd_write_update(void);

    RC_Cmd rc; // in case for original RC use, will be deleted after control interface is settled

    WB6_Parameter &param;
    Cmd &cmd; // robot_task api, should be set as function

    Watchdog &wdg;

    MotorPIDParam _motorPID;
    Run_Medium_Package_Status_t medium_status;

private:                                          // function package
    Init_Handle initHandle;                       // 初始化模态
    Leg_Wheel_Drive_Class leg_wheel_drive_handle; // 轮组自由行走模态
    Crash_Class crash_handle;

private:
    bool start_reset = false;
};
