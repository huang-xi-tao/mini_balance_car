#pragma once
#include <math.h>
#include <string.h>

template<typename T, uint8_t Size, uint8_t Dim = 2>
class LinearFit{
public:
    LinearFit(void)
    {
        memset(coefs, 0, sizeof(T)*Dim);
        this->reset();
    }

    void reset(void)
    {
        memset(x, 0, sizeof(T)*Size*Dim);
        memset(y, 0, sizeof(T)*Size);
        this->index = 0;
    }

    void update(const T input[Dim], const T result)
    {
        memcpy(x[index], input, sizeof(T)*Dim);
        y[index] = result;
        index++;
    }

    T* calc_result(const float err_threshold, float& error)
    {
        if(!index)  return NULL;

        T A[Dim][Dim];
        T b[Dim];
        memset(A, 0, sizeof(T)*Dim*Dim);
        memset(b, 0, sizeof(T)*Dim);
        memset(coefs, 0, sizeof(T)*Dim);

        for(uint8_t i = 0; i < index; i++)
        {
            for(uint8_t j = 0; j < Dim; j++)
            {
                for(uint8_t k = 0; k < Dim; k++)
                    A[j][k] += x[i][j]*x[i][k];
                b[j] += x[i][j]*y[i];
            }
        }

        //coefs = inv[A'A]*A'b
        float det = 1/(A[0][0] * A[1][1] - A[0][1] * A[1][0]);
        coefs[0] = det * ( A[1][1] * b[0] - A[1][0] * b[1]);
        coefs[1] = det * (-A[0][1] * b[0] + A[0][0] * b[1]);

        error = this->calc_error(this->coefs);
        return error < err_threshold ? coefs : NULL;
    }

    float calc_error(T coefs[Dim])
    {
        if(!index)
            return 0;

        T result = 0;
        for(uint8_t i = 0; i < index; i++)
        {
            T calc = 0;
            for(uint8_t j = 0; j < Dim; j++)
                calc += coefs[j]*x[i][j];
            T err = y[i] - calc;
            result += err*err;
        }

        return sqrtf(result/index);
    }

private:
    T x[Size][Dim];
    T y[Size];
    uint8_t index;

    T coefs[Dim];
};
