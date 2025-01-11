#pragma once

class Status_Flag_Class
{
public:
    Status_Flag_Class()
    {
    }
    bool balance = false;
    bool left_offground = false;
    bool leg_on = false;

    bool ok = false; // robot can run
    bool offground = false;
    bool offground_stable = false;
    bool pitch_limit = false;
    bool right_offground = false;
    bool sdk_debug = false;
};
