#pragma once
#ifndef POWER_HPP__
#define POWER_HPP__
#include <stdint.h>

class Power_Base
{
protected:
    Power_Base(const float Vin):Vin(Vin), Current(0), Capacitor(0), bat_percent(100){}

public:
    float            Vin;
    float        Current;
    float      Capacitor;
    uint8_t  bat_percent;
};

class Power_Dummy : public Power_Base
{
public:
    Power_Dummy(const float Vin = 32.0f):Power_Base(Vin){}
};

#endif