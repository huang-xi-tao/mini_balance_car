#pragma once

#include <stdint.h>
#include <math.h>

template <typename VAL>
class Ramp
{
public:
    Ramp() : val(0), t(0), T(1) {}

    void result(float dt, float T, float start, float end)
    {
        this->t += dt;
        float result = start + this->t / T * (end - start);
    }

    void reset(const VAL T)
    {
        this->val = 0.0f;
        this->t = 0.0f;
        this->T = T;
    }

    void setVal(const VAL T, const VAL input)
    {
        this->val = input;
        this->t = input * T;
        this->T = T;
    }

    void setT(const VAL T)
    {
        this->T = T;
    }

    bool update(const VAL dt)
    {
        this->t += dt;
        if (this->t > this->T)
        {
            this->val = 1.0;
            return true;
        }

        this->val = this->t / this->T;
        return false;
    }

    VAL getResult(const VAL start, const VAL end)
    {
        return start + val * (end - start);
    }

    VAL getDResult(const VAL start, const VAL end)
    {
        if (val <= 0 || val >= 1)
            return 0;
        else
            return (end - start) / T;
    }

    VAL get_smooth_result(const VAL start, const VAL end)
    {
        if (val <= 0.25f)
        {
            return start + 3.25 * val * (end - start);
        }
        else
        {
            return start + 0.7f * (end - start) + 0.25 * val * (end - start);
        }
    }
    VAL get_head_result(const VAL start, const VAL end)
    {
        // if(val <= 0.5f)
        // {
        //     return start + 3.5 * val*(end - start);
        // }
        // else
        // {
        return start + val * (end - start);
        // }
    }

    bool finish(void)
    {
        return (val >= 1);
    }

    VAL val;
    VAL t;
    VAL T;

private:
};
