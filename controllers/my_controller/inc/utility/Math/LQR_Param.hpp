#pragma once

#include "MathFunc.hpp"

struct LQR2_param{
    LQR2_param(const float& val): Val(val){}
    
    float getVal(const float& input)
    {
        (void)input;
        return Val;
    }

private:
    float Val;
};
