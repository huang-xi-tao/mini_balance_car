#include "Watchdog.hpp"

bool Watchdog::check(uint32_t flag, error_level_t level)
{
    switch (level)
    {
        case CRITICAL:
            return *error          & flag;
        case SEVERE:
            return *severe_warning & flag;
        case NORMAL:
            return *warning        & flag;
        
    }
    return false;
}


void Watchdog::setFlag(uint32_t flag, error_level_t level)
{
    if(!this->enable)   return;

    switch (level)
    {
        case CRITICAL:
            *error          |= flag;
            break;
        case SEVERE:
            *severe_warning |= flag;
            break;
        case NORMAL:
            *warning        |= flag;
            break;
    }
} 

void Watchdog::clearFlag(uint32_t flag, error_level_t level)
{
    switch (level)
    {
        case CRITICAL:
            *error          &= (~flag);
            break;
        case SEVERE:
            *severe_warning &= (~flag);
            break;
        case NORMAL:
            *warning        &= (~flag);
            break;
    }
}

void Watchdog::setMotorError(const uint8_t error, const uint8_t id)
{
    if(!this->enable)   return;
    motor_error[id] = error;
}
