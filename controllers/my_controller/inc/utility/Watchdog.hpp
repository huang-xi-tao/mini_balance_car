#pragma once
#ifndef WDG_HPP__
#define WDG_HPP__
#include <stdint.h>

enum
{
    RC_KILL = 1,
    SDK_KILL = 1 << 1,
    PITCH = 1 << 2,
    ROLL = 1 << 4,
    SPLIT = 1 << 6,

    RESET_FAIL = 1 << 8,
    FIRMWARE_UPDATE = 1 << 9,
    VERSION_ERROR = 1 << 10,
    PARAM_ERROR = 1 << 11,

    POWER_ERROR = 1 << 12,
    RC_ERROR = 1 << 13,
    ATTITUDE_ERROR = 1 << 14,

    MOTOR_DISCONNECT = 1 << 15,
    MOTOR_ERROR = 1 << 21,

    KILL_MASK = 0b11,                // will attempt to recover after clear
    FALLOVER_MASK = 0b111111 << 2,   // will attempt to recover
    FATAL_ERROR_MASK = 0x7FFFF << 8, // will not attempt to recover
};

enum
{
    POWER_LOW = 1,
    MOTOR_WARNING = 1 << 1,
    BMS_OFFLINE = 1 << 7,
    CAPACITOR_LOW = 1 << 8,
    LEG_OVERLOAD = 1 << 9,
    WHEEL_OVERSPEED = 1 << 10,
    ATTITUDE_WARNING = 1 << 11,

    PARAM_UPDATE_REQUEST = 1 << 12,
    PARAM_WARNING = 1 << 13,
    FIRMWARE_OUT_OF_DATE = 1 << 14,
    POWER_CHARGING = 1 << 15,
    OVER_CURRENT = 1 << 16,
    MOTOR_WARNING_MASK = 0b111111 << 1
};

class Watchdog
{

private:
    bool enable;
    uint8_t flag[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    uint32_t *severe_warning;
    uint32_t *warning;
    uint8_t *motor_error;

public:
    enum error_level_t
    {
        CRITICAL = 0,
        SEVERE = 2,
        NORMAL = 3
    };

    void start(void)
    {
        this->enable = true;
    }

    bool Error(void) { return *error; }
    bool Warning(void) { return (*severe_warning) || (*warning); }
    bool SevereWarning(void) { return *severe_warning; }

    uint32_t getError(void) { return *error; }
    uint32_t getWarning(void) { return *severe_warning; } //{return (uint64_t)(*severe_warning)<<32 | (*warning);}

    bool check(uint32_t flag, error_level_t level = CRITICAL);
    void setFlag(uint32_t flag, error_level_t level = CRITICAL);
    void clearFlag(uint32_t flag, error_level_t level = CRITICAL);
    void setMotorError(const uint8_t error, const uint8_t id);

    float overspeed_dt = 0;
    uint32_t *error;

    Watchdog() : flag{0},
                 error((uint32_t *)&flag[0]),
                 severe_warning((uint32_t *)&flag[10]),
                 warning((uint32_t *)&flag[14]),
                 motor_error(&flag[4])
    {
        this->enable = false;
    }
};

#endif