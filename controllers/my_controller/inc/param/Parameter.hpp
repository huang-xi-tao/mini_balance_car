#ifndef _PARAMETER_H_
#define _PARAMETER_H_
#pragma once
#include "Parameter_Core.hpp"
#include "Parameter_Device.hpp"
#include "Parameter_LQR.hpp"
#include "param_wheelBipedRobot.hpp"
#include "param_DIABLO.hpp" //should use a
typedef struct
{
    Param_Core_t core;
    Param_LQR_Set lqr;
    Param_Kin_t kin;
} WB6_Parameter_Core;

class WB6_Parameter
{
public:
    WB6_Parameter(
        const Param_Core_t &core,
        const Param_LQR_Set &set,
        const Param_Kin_t &kin) : init(false), core(core), lqr(set), kin(kin){

                                                                     };

    void *Param_Device;

public:
    bool init;
    int8_t knee_direct;

    const Param_Core_t &core;
    const Param_LQR_Set &lqr;
    const Param_Kin_t &kin;
    Param_IMU_Bias imu_bias;
    Param_Mass_Property mass;
};

#endif
