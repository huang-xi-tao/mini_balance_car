#pragma once
class Ctrl_Output_Class
{
public:
    float forward_out = 0;
    float yaw_out = 0;
    float knee_out = 0;
    float tilt_out = 0;

    void Clear_Output(void)
    {
        forward_out = 0;
        yaw_out = 0;
        knee_out = 0;
        tilt_out = 0;
    }

    void Clear_Forward_Output(void)
    {
        forward_out = 0;
    }
    void Clear_Yaw_Output(void)
    {
        yaw_out = 0;
    }
};