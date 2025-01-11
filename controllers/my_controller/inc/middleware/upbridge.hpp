#pragma once
#ifndef UP_BRIDGE_HPP__
#define UP_BRIDGE_HPP__
#include "bridge.hpp"
#include "Quaternion.hpp"
#include "Joint_motor.hpp"
class Upbridge_Class
{
public:
    Upbridge_Class(){};
    ~Upbridge_Class(){};
    void Get_Angular_Velocity(float *rotation);
    void Get_Acceleration(float *acceleration);
    void Get_Motor_Torque(unsigned char id, float *torque);
    void Get_Motors_Velocity(unsigned char id, float *velocity);
    void Get_Motors_Angle(unsigned char id, float *angle);
    void Get_Battery_Voltage(float *voltage);
    void Get_Battery_Current(float *current);
    void Get_Battery_Capacity(float *capacity);
    void Get_Quaternion(Quaternion *qua);
    void Set_Motors_Torque(unsigned char id, float torque);
    void Set_Motors_Angle(unsigned char id, float angle_set);
    void Get_Motor(Joint_motor *joint);
    void Get_TransformBase2imu(float *transform);

    Up2Down_Class up2down;
    const Down2Up_Class *down2up_ptr;

private:
};

#endif