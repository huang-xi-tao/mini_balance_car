#ifndef MIDDLEWARE_HPP__
#define MIDDLEWARE_HPP__
#pragma once
// middleware for 6 degree leg-wheel robot
#include "upbridge.hpp"
#include "downbridge.hpp"
#include "bridge.hpp"
#define JOINT_SERIES_1
#ifdef JOINT_SERIES_1
#include "param_DIABLO.hpp"
#endif

class Middleware_Class
{
public:
    Middleware_Class() : downbridge(), upbridge()
    {
        // downbridge_ptr = &downbridge;
        // upbridge_ptr = &upbridge;
        // downbridge_ptr->up2down_ptr = &upbridge_ptr->up2down;
        // upbridge_ptr->down2up_ptr = &downbridge_ptr->down2up;
        downbridge.up2down_ptr = &upbridge.up2down;
        upbridge.down2up_ptr = &downbridge.down2up;
    };
    // Upbridge_Class *upbridge_ptr = NULL;
    // Downbridge_Class *downbridge_ptr = NULL;
    Downbridge_Class downbridge;
    Upbridge_Class upbridge;

private:             // Add more variables and their corresponding
protected:           // write your own task, and declare them here. Written in the user main thread.
    void Main_Run(); // The main task
};

#endif
