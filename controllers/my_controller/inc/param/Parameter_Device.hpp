#ifndef _PARAMETER_USER_H_
#define _PARAMETER_USER_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/*
 *  Device parameter storage struct
 * {
 *      head  : 1      byte
 *      len   : 1      byte
 *      data  : $(len) bytes
 *      crc16 : 2      bytes
 * }
 * TODO: How to get derived class size and pointer correctly on different machines
 */

#define PARAM_STD_HEAD 0xA5

#pragma pack(1)
class Param_Device
{
protected:
    Param_Device(const size_t size) : size(size - sizeof(Param_Device)) {}

    void reset(void)
    {
        // memset(data, 0, size);
    }

    size_t size;
    void *data;

public:
    size_t store_size(void) const
    {
        return size + 4;
    }

    size_t OTA_store_size(void) const
    {
        return size;
    }

    bool load(const uint8_t data[], const uint8_t head = PARAM_STD_HEAD);
    void OTA_Fill(uint8_t output[]);
    void complete(uint8_t output[], const uint8_t head = PARAM_STD_HEAD) const;
};

class Param_Robot_Info : public Param_Device
{
public:
    Param_Robot_Info(void) : Param_Device(
                                 sizeof(Param_Robot_Info))
    {
        this->data = (void *)&HW_Version;
        this->reset();
    }

    unsigned int HW_Version;
    unsigned int SW_Version;
    unsigned int SN;
    unsigned int Product_Date;
    unsigned int Client_Code;
};

class Param_IMU_Bias : public Param_Device
{
public:
    Param_IMU_Bias(void) : Param_Device(
                               sizeof(Param_IMU_Bias))
    {
        this->data = (void *)&x;
        this->reset();
    }

    float x;
    float y;
    float z;
};

class Param_Mass_Property : public Param_Device
{
public:
    Param_Mass_Property(void) : Param_Device(
                                    sizeof(Param_Mass_Property))
    {
        this->data = (void *)&body_d;
        this->reset();
    }

    float body_d;
    float body_ang;
    float head_x;
    float head_z;
};

class Param_Motor_Init_Pos : public Param_Device
{
public:
    Param_Motor_Init_Pos(void) : Param_Device(
                                     sizeof(Param_Motor_Init_Pos))
    {
        this->reset();
    }
    float knee;
    float wheel;
};

/*OTA class*/
class Param_OTA_INIT : public Param_Device
{
public:
    Param_OTA_INIT(void) : Param_Device(
                               sizeof(Param_OTA_INIT))
    {
        this->data = (void *)&key;
        this->reset();
    }
    uint8_t key[16];
    /*OTA message
    Please fill in the last section if you want to go further*/
    uint32_t Hard_Version;
    uint32_t Boot_Version;
    uint32_t Soft_Version;
    uint32_t Device_ID;
    uint32_t model[4];
    uint32_t APP_Ready;
};

#pragma pack()

#endif
