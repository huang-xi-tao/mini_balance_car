#ifndef DOWN_BRIDGE_HPP__
#define DOWN_BRIDGE_HPP__
#pragma once
#include "bridge.hpp"

class Downbridge_Class
{

public: // Downbridge functions should be written, which is the driver
    Downbridge_Class(){};
    ~Downbridge_Class(){};
    void Set_Gyro(double x, double y, double z); // The driver to get IMU,and the conversion.
    void Set_Accl(double x, double y, double z);
    // id 0~7  left:abad=0 hip knee wheel  right:abad=4 hip knee wheel
    void Set_Motor(float torque /*Nm*/, float velocity /*rad/s*/, float angle /*rad*/, unsigned char id); 
    void Set_Camera();                                                                                    // TBD
    void Set_Infra();                                                                                     // TBD
    void Set_Battery(float voltage, float current, float capacity);
    void Set_Quaternion(float w, float x, float y, float z);
    void Set_Forward_Cmd(int forward);
    void Set_TransformBase2imu(float x = 0.0, float y = 0.0, float z = 0.0, float roll = 0.0, float pitch = 0.0, float yaw = 0.0);

    void Get_Motor_Torque(float &torque, unsigned char id); 
    void Get_Motor_Pos(float &pos, unsigned char id);
    Down2Up_Class down2up;
    const Up2Down_Class *up2down_ptr;
};
#endif