#include "downbridge.hpp"

void Downbridge_Class::Set_Gyro(double x, double y, double z) // get IMU,and the conversion.
{                                                             // add qicibianhuan
    down2up.gyro[0] = x;
    down2up.gyro[1] = y;
    down2up.gyro[2] = z;
}
void Downbridge_Class::Set_Accl(double x, double y, double z)
{
    down2up.accl[0] = x;
    down2up.accl[1] = y;
    down2up.accl[2] = z;
}
void Downbridge_Class::Set_Motor(float torque /*Nm*/, float velocity /*rad/s*/, float angle /*rad*/, unsigned char id)
{
    down2up.torque_fbk[id] = torque;
    down2up.velocity_fbk[id] = velocity;
    down2up.angle_fbk[id] = angle;
}

void Downbridge_Class::Set_Battery(float voltage, float current, float capacity)
{
    down2up.battery_vol = voltage;
    down2up.battery_curr = current;
    down2up.battery_capacity = capacity;
}

void Downbridge_Class::Get_Motor_Torque(float &torque, unsigned char id)
{
    torque = up2down_ptr->torque_set[id];
}

void Downbridge_Class::Get_Motor_Pos(float &pos, unsigned char id)
{
    pos = up2down_ptr->angle_set[id];
}
// TBD
void Downbridge_Class::Set_Camera()
{
}
// TBD
void Downbridge_Class::Set_Infra()
{
}

void Downbridge_Class::Set_Quaternion(float w, float x, float y, float z)
{
    down2up.q[0] = w;
    down2up.q[1] = x;
    down2up.q[2] = y;
    down2up.q[3] = z;
}

void Downbridge_Class::Set_TransformBase2imu(float x, float y, float z, float roll, float pitch, float yaw)
{
    down2up.imu_transform_to_base[0] = x;
    down2up.imu_transform_to_base[1] = y;
    down2up.imu_transform_to_base[2] = z;
    down2up.imu_transform_to_base[3] = roll;
    down2up.imu_transform_to_base[4] = pitch;
    down2up.imu_transform_to_base[5] = yaw;
}
