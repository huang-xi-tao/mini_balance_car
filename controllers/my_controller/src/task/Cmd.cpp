#include "Cmd.hpp"
#include <math.h>
#include <string.h>

void Command::Channel::input_rev(float val_in, const float dt) // reverse
{
    this->d_val = val_in;
    this->val += val_in * dt;
    if (val > M_PI)
    {
        val -= 2 * M_PI;
        rev += 2;
    }
    else if (val < -M_PI)
    {
        val += 2 * M_PI;
        rev -= 2;
    }
}

void Command::Channel::input_limit(float val_in, const float dt,
                                   const float up_limit_d, const float low_limit_d, const float up_limit, const float low_limit)
{
    float max_val_in, min_val_in;
    switch (ctrl_mode)
    {
    case INPUT_POS:
        if (val_in > up_limit)
        {
            val = up_limit;
            d_val = 0;
            return;
        }
        else if (val_in < low_limit)
        {
            val = low_limit;
            d_val = 0;
            return;
        }

        max_val_in = val + up_limit_d * dt;
        min_val_in = val + low_limit_d * dt;
        if (val_in > max_val_in)
        {
            d_val = up_limit_d;
            val = max_val_in;
        }
        else if (val_in < min_val_in)
        {
            d_val = low_limit_d;
            val = min_val_in;
        }
        else
        {
            d_val = (val_in - val) / dt;
            val = val_in;
        }
        break;
    case INPUT_VEL:
        if (val_in > up_limit_d)
            val_in = up_limit_d;
        else if (val_in < low_limit_d)
            val_in = low_limit_d;
        this->val += val_in * dt;
        if (this->val > up_limit)
        {
            this->val = up_limit;
            this->d_val = 0;
        }
        else if (this->val < low_limit)
        {
            this->val = low_limit;
            this->d_val = 0;
        }
        else
            this->d_val = val_in;
        break;
    }
}

RC_Cmd::RC_Cmd(void)
{
    memset(this, 0, sizeof(RC_Cmd));
}
