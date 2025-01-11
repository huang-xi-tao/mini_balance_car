#include "robot_likeAscento.hpp"
#include "PID.hpp"
#include "stdio.h"

static Power_Dummy pDummy;

Robot_Class::Robot_Class(WB6_Parameter &param, Middleware_Class &middle) : param(param),
                                                                           legL(0, &param), legR(1, &param),
                                                                           cmd_num(8),
                                                                           posture(param), wdg()
{
    middle_ptr = &middle; // for communication with middleware
    flag_ptr = &flag;
    legL_ptr = &legL;
    legR_ptr = &legR;
    power_ptr = &power;
    posture_ptr = &posture;
    param_ptr = &param;
    power = pDummy;
    flag.left_offground = flag.right_offground = flag.offground = false;

    posture.Pass_Left_Leg_Ptr(&legL);
    posture.Pass_Right_Leg_Ptr(&legR);

    legL.Start();
    legR.Start();
#if sim_to_real
    param.knee_direct = -1;
    param.imu_bias.y = 0.15f;
#else
    param.knee_direct = 1;
    param.imu_bias.y = 0.0f;
#endif
}

void Robot_Class::Tail_Up()
{
    this->Tail_motor.setPosition(param_ptr->core.TAIL_UP_POS);
}

void Robot_Class::Tail_Down(float angle_set)
{
    this->Tail_motor.setPosition(angle_set);
}

void Robot_Class::Tail_Down_FirstTime()
{
    cmd.tail_angle.val = param.core.TAIL_DOWN_POS_FIRST;
    this->Tail_motor.setPosition(param.core.TAIL_DOWN_POS_FIRST);
}

void Robot_Class::Robot_Update(float dt)
{
    this->stamp += dt;

    middle_ptr->upbridge.Get_Motor_Torque(0, &legL.knee.torque);
    middle_ptr->upbridge.Get_Motor_Torque(1, &legL.wheel.torque);
    middle_ptr->upbridge.Get_Motor_Torque(2, &legR.knee.torque);
    middle_ptr->upbridge.Get_Motor_Torque(3, &legR.wheel.torque);
    middle_ptr->upbridge.Get_Motor_Torque(4, &Tail_motor.torque);

    middle_ptr->upbridge.Get_Motors_Angle(0, &legL.knee.angle);
    middle_ptr->upbridge.Get_Motors_Angle(1, &legL.wheel.angle);
    middle_ptr->upbridge.Get_Motors_Angle(2, &legR.knee.angle);
    middle_ptr->upbridge.Get_Motors_Angle(3, &legR.wheel.angle);
    middle_ptr->upbridge.Get_Motors_Angle(4, &Tail_motor.angle);

    middle_ptr->upbridge.Get_Motors_Velocity(0, &legL.knee.angle_vel);
    middle_ptr->upbridge.Get_Motors_Velocity(1, &legL.wheel.angle_vel);
    middle_ptr->upbridge.Get_Motors_Velocity(2, &legR.knee.angle_vel);
    middle_ptr->upbridge.Get_Motors_Velocity(3, &legR.wheel.angle_vel);
    middle_ptr->upbridge.Get_Motors_Velocity(4, &Tail_motor.angle_vel);

    legL.knee.update(legL.knee.angle, legL.knee.angle_vel, legL.knee.torque);
    legL.wheel.update(legL.wheel.angle, legL.wheel.angle_vel, legL.wheel.torque);

    legR.knee.update(legR.knee.angle, legR.knee.angle_vel, legR.knee.torque);
    legR.wheel.update(legR.wheel.angle, legR.wheel.angle_vel, legR.wheel.torque);
    Tail_motor.update(Tail_motor.angle, Tail_motor.angle_vel, Tail_motor.torque);
    this->middle_ptr->upbridge.Get_Angular_Velocity(this->posture.rotation);
    this->middle_ptr->upbridge.Get_Acceleration(this->posture.accl);
    this->middle_ptr->upbridge.Get_Quaternion(&this->posture._quaternion);
    this->middle_ptr->upbridge.Get_TransformBase2imu(this->posture._body2imu);

    for (int id = 0; id < MOTOR_NUM; id++)
    {
        this->middle_ptr->upbridge.Get_Motor_Torque(id, &this->posture._tauMotor[id]);
        this->middle_ptr->upbridge.Get_Motors_Angle(id, &this->posture._qMotor[id]);
        this->middle_ptr->upbridge.Get_Motors_Velocity(id, &this->posture._dqMotor[id]);
    }
    this->posture.attitude_update(dt);

    this->legL.Update_Leg(posture.head_pitch, posture.d_head_pitch, &this->posture._qbodyEst[0], &this->posture._dqbodyEst[0], dt);
    this->legR.Update_Leg(posture.head_pitch, posture.d_head_pitch, &this->posture._qbodyEst[2], &this->posture._dqbodyEst[2], dt);
    // this->legL.Force_Arm_Estimate(posture._quaternion, posture.leg_angle, posture.vel_forward, posture.vel_yaw, posture.roll, flag_ptr->offground);
    // this->legR.Force_Arm_Estimate(posture._quaternion, posture.leg_angle, posture.vel_forward, posture.vel_yaw, posture.roll, flag_ptr->offground);
    // this->legR.Set_Load_Psc(1 - this->legL.load_psc);

    float percent = 100;
    power.bat_percent = percent;
    middle_ptr->upbridge.Get_Battery_Voltage(&power.Vin);

    posture.roll_estimate(legL.len, legR.len, param.core.WHEEL_DISTANCE);
    posture.pitch_estimate();
    posture.wheel_estimate(param.core.WHEEL_RADIUS, dt, legL, legR);
    posture.tilt_estimate(param, legL, legR);
    posture.update_heading();

    posture.leg_len = (legL.len + legR.len) * 0.5;
    posture.leg_angle = (legL.angle + legR.angle) * 0.5;
    posture.height = (legL.len * cosf(posture._pitch) + legR.len * cosf(posture._pitch)) * 0.5f; // robot.posture_ptr->leg_len * robot.posture_ptr->cos_tilt;

    float max_vel = (legL.wheel.max_vel + legR.wheel.max_vel) * 0.5;
    if (mabs(posture.vel_forward) > max_vel * param.core.WHEEL_RADIUS * 0.8f)
    {
        wdg.overspeed_dt += dt;
        if (wdg.overspeed_dt > 0.5f)
            wdg.setFlag(WHEEL_OVERSPEED, Watchdog::SEVERE);
        else
        {
            wdg.overspeed_dt = 0;
            if (mabs(posture.vel_forward) < param.core.FORWARD_MAX_VEL)
                wdg.clearFlag(WHEEL_OVERSPEED, Watchdog::SEVERE);
        }
    }
}

void Robot_Class::Clear_Robot_Flag(void)
{
    flag.balance = false;
    flag.left_offground = false;
    flag.leg_on =
        flag.ok = false; // robot can run
    flag.offground = false;
    flag.offground_stable = false;
    flag.pitch_limit = false;
    flag.right_offground = false;
    flag.sdk_debug = false;
}

void Robot_Class::Set_Robot_Flag(flag_type_t input_flag, bool status)
{
    switch (input_flag)
    {
    case BALANCE:
        flag.balance = status;
        break;
    case LEFT_OFFGROUND:
        flag.left_offground = status;
        break;
    case LEG_ON:
        flag.leg_on = status;
        break;
    case OK:
        flag.ok = status;
        break;
    case OFFGROUND:
        flag.offground = status;
        break;
    case OFFGROUND_STABLE:
        flag.offground_stable = status;
        break;
    case PITCH_LIMIT:
        flag.pitch_limit = status;
        break;
    case RIGHT_OFFGROUND:
        flag.right_offground = status;
        break;
    case SDK_DEBUG:
        flag.sdk_debug = status;
        break;
    }
}

void Robot_Class::Set_Output(Ctrl_Output_Class left_out, Ctrl_Output_Class right_out)
{
    float wheelL_out = 0;
    float wheelR_out = 0;
    float max_out_L = 0, max_out_R = 0;
    float wheel_weight = 3.0;

    wheelL_out = (left_out.forward_out - left_out.yaw_out); // Todo
    if (flag.leg_on)
    {
        max_out_L = 100.f;
        bound(wheelL_out, max_out_L);
    }
    legL.knee.setTorque(left_out.knee_out);
    legL.wheel.setTorque(wheelL_out);

    wheelR_out = (right_out.forward_out + right_out.yaw_out);
    if (flag.leg_on)
    {
        max_out_R = 100.f;
        bound(wheelR_out, max_out_R);
    }
    legR.knee.setTorque(right_out.knee_out);
    legR.wheel.setTorque(wheelR_out);
    Tail_motor.setPosition(cmd.tail_angle.val);

    middle_ptr->upbridge.Set_Motors_Torque(0, legL.knee.torque_set);
    middle_ptr->upbridge.Set_Motors_Torque(1, legL.wheel.torque_set);
    middle_ptr->upbridge.Set_Motors_Torque(2, legR.knee.torque_set);
    middle_ptr->upbridge.Set_Motors_Torque(3, legR.wheel.torque_set);
    middle_ptr->upbridge.Set_Motors_Angle(4, Tail_motor.angle_set);
}
