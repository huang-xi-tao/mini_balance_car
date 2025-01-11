#include "upbridge.hpp"

// float can be converted to const float, but const float cannot be converted to float
// These functions can be used in the robot_class initialization
void Upbridge_Class::Get_Angular_Velocity(float *rotation)
{
    rotation[0] = down2up_ptr->gyro[0];
    rotation[1] = down2up_ptr->gyro[1];
    rotation[2] = down2up_ptr->gyro[2];
}
void Upbridge_Class::Get_Acceleration(float *accl)
{
    accl[0] = down2up_ptr->accl[0];
    accl[1] = down2up_ptr->accl[1];
    accl[2] = down2up_ptr->accl[2];
}

void Upbridge_Class::Get_Motor(Joint_motor *motor)
{
    for (unsigned char id = 0; id < MOTOR_NUM; id++)
    {
        motor[id].torque = down2up_ptr->torque_fbk[id];
        motor[id].angle_vel = (down2up_ptr->velocity_fbk[id]);
        motor[id].angle = (down2up_ptr->angle_fbk[id]);
    }
}

void Upbridge_Class::Get_Motor_Torque(unsigned char id, float *torque)
{
    *torque = down2up_ptr->torque_fbk[id];
}
void Upbridge_Class::Get_Motors_Velocity(unsigned char id, float *velocity)
{
    *velocity = (down2up_ptr->velocity_fbk[id]);
}
void Upbridge_Class::Get_Motors_Angle(unsigned char id, float *angle)
{
    *angle = (down2up_ptr->angle_fbk[id]);
}
void Upbridge_Class::Set_Motors_Torque(unsigned char id, float torque)
{
    up2down.torque_set[id] = torque;
}
void Upbridge_Class::Set_Motors_Angle(unsigned char id, float angle_set)
{
    up2down.angle_set[id] = angle_set;
}
void Upbridge_Class::Get_Battery_Voltage(float *voltage)
{
}

void Upbridge_Class::Get_Quaternion(Quaternion *qua)
{
    qua->w = down2up_ptr->q[0];
    qua->x = down2up_ptr->q[1];
    qua->y = down2up_ptr->q[2];
    qua->z = down2up_ptr->q[3];
}

void Upbridge_Class::Get_TransformBase2imu(float *transform)
{

    // float transform[6] = {0};
    for (unsigned char i = 0; i < 6; i++)
    {
        transform[i] = down2up_ptr->imu_transform_to_base[i];
    }
}
