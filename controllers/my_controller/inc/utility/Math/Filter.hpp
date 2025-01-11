#pragma once
#include <stdint.h>
#include <math.h>

template<typename T>
class LPFilter2{
public:
    LPFilter2(void)
    {
        this->data[0] = this->data[1] = 0;
    }

    LPFilter2(const T sample_freq, const T cutoff_freq)
    {
        this->data[0] = this->data[1] = 0;
        this->init(sample_freq, cutoff_freq);
    }

    void init(const T sample_freq, const T cutoff_freq)
    {
        T fr = sample_freq / cutoff_freq;
        T ohm = tanf(M_PI / fr);
        T c = 1.0f + 2.0f *cosf(M_PI / 4.0f) * ohm + ohm*ohm;
        this->b0 = ohm*ohm / c;
        this->b1 = 2.0f * this->b0;
        this->b2 = this->b0;
        this->a1 = 2.0f * (ohm*ohm - 1.0f) / c;
        this->a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm*ohm) / c;
    }

    T update(const T input)
    {
        if(!isfinite(input))    
            return this->data[0]*(this->b0 + this->b1) + this->data[1]*this->b2;

        T delay = input - this->data[0] * this->a1 - this->data[1] * this->a2;

        if(!isfinite(delay))
            delay = input;
        
        T output = delay * this->b0 +  this->data[0] * this->b1 + this->data[1] * this->b2;

        this->data[1] = this->data[0];
        this->data[0] = delay;

        return output;
    }

private:
    T a1;
    T a2;
    T b0;
    T b1;
    T b2;

    T data[2];
};
