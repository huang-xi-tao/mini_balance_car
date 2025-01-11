#pragma once
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "MathConst.h"

template <typename T>
static inline T mabs(const T &input)
{
    return (input > 0) ? input : -input;
}

template <typename T>
static inline void bound(T &input, const T max, const T min)
{
    if (input > max)
        input = max;
    else if (input < min)
        input = min;
}

template <typename T>
static inline void bound(T &input, const T max)
{
    bound(input, max, -max);
}

template <typename T>
static inline T boundOutput(const T input, const T max, const T min)
{
    T output;
    if (input < max && input > min)
        output = input;
    else if (input >= max)
        output = max;
    else
        output = min;

    return output;
}

template <typename T>
static inline T boundOutput(const T input, const T max)
{
    return boundOutput(input, max, -max);
}

template <typename T>
static inline T vector_norm(const T v[], const uint8_t length)
{
    uint8_t i;
    T norm = 0.0f;
    for (i = 0; i < length; i++)
        norm += v[i] * v[i];
    return sqrtf(norm);
}

template <typename T>
static inline void vector_normalize(T v[], const uint8_t length)
{
    uint8_t i;
    T norm = vector_norm(v, length);
    for (i = 0; i < length; i++)
        v[i] /= norm;
}

template <typename T>
static inline void vector3_cross(const T a[3], const T b[3],
                                 T result[3])
{
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

template <typename T>
static inline T vector_dot(const T *a, const T *b, int size)
{
    T result = 0;
    for (int id = 0; id < size; id++)
        result += a[id] * b[id];
    return result;
}

/**
 * @source from MatrixMath.cpp from Arduino
 * @brief for known size matrix multiplication, no check
 * A = input matrix (m x p)
 * x = input vector (p x n)
 * m = number of rows in A
 * n = number of columns in A = number of rows in x
 * b = output matrix = A*x
 */
template <typename T>
static inline void matrix33_multiply_vector3(const T A[3][3], const T x[3], T b[3])
{
    uint8_t i, j;

    for (i = 0; i < 3; i++)
    {
        b[i] = 0.0f;
        for (j = 0; j < 3; j++)
            b[i] += A[i][j] * x[j];
    }
}

template <typename T>
static inline void matrix33_multiply_vector3(const T *A, const T x[3], T b[3])
{
    uint8_t i, j;

    for (i = 0; i < 3; i++)
    {
        b[i] = 0.0f;
        for (j = 0; j < 3; j++)
            b[i] += *(A + i * 3 + j) * x[j];
    }
}

template <typename T>
static inline void matrix33MultiplyMatrix33(const T A[3][3], const T B[3][3], T C[3][3])
{
    uint8_t i, j, k;

    for (i = 0; i < 3; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            C[i][j] = 0;
            for (k = 0; k < 3; ++k)
                C[i][j] += A[i][k] * B[k][j];
        }
    }
}

template <typename T>
static inline void matrix3inverse(const T A[3][3], T B[3][3])
{
    T det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) - A[0][1] * (A[1][0] * A[2][2] - A[2][0] * A[1][2]) + A[0][2] * (A[1][0] * A[2][1] - A[2][0] * A[1][1]);
    B[0][0] = A[1][1] * A[2][2] - A[2][1] * A[1][2];
    B[1][0] = A[2][0] * A[1][2] - A[1][0] * A[2][2];
    B[2][0] = A[1][0] * A[2][1] - A[2][0] * A[1][1];
    B[0][1] = A[2][1] * A[0][2] - A[0][1] * A[2][2];
    B[1][1] = A[0][0] * A[2][2] - A[2][0] * A[0][2];
    B[2][1] = A[2][0] * A[0][1] - A[0][0] * A[2][1];
    B[0][2] = A[0][1] * A[1][2] - A[1][1] * A[0][2];
    B[1][2] = A[1][0] * A[0][2] - A[0][0] * A[1][2];
    B[2][2] = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            B[i][j] = B[i][j] / det;
}

template <typename T>
static inline void matrixTranspose(const T A[3][3], T B[3][3])
{
    uint8_t i, j;

    for (i = 0; i < 3; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            B[j][i] = 0;
            B[j][i] = A[i][j];
        }
    }
}

template <typename T>
static inline void deadzone(T &input, const T deadzone)
{
    if (input > deadzone)
        input -= deadzone;
    else if (input < -deadzone)
        input += deadzone;
    else
        input = 0;
}

template <typename T, uint8_t Size = 16>
class Circular_Differentiator
{
public:
    Circular_Differentiator() : init(false), index(0), val_buffer{0}, stamp_buffer{0} {}

    T update(const T val, const T dt)
    {
        uint8_t curr_index = index % Size, prev_index = (++index) % Size;
        float time_err = 0;

        val_buffer[curr_index] = val;
        // stamp_buffer[curr_index] = stamp;

        if (index < Size)
            return 0;
        init = true;
        time_err = dt;
        if (time_err <= 0.001)
            time_err = 0.001;

        return (val - val_buffer[prev_index]) / (time_err);
    }

private:
    bool init;
    uint32_t index;

    T val_buffer[Size];
    T stamp_buffer[Size];
};

template <typename T, uint8_t Size = 16>
class Circular_Intergrator
{
public:
    Circular_Intergrator() : init(false), index(0), val_buffer{0}, stamp_buffer{0} {}

    T update(const T val, const T stamp)
    {
        uint8_t curr_index = index % Size, prev_index = (++index) % Size, time_err = 0;

        val_buffer[curr_index] = val;
        stamp_buffer[curr_index] = stamp;

        if (index < Size)
            return 0;
        init = true;
        time_err = stamp - stamp_buffer[prev_index];
        if (time_err <= 0.001)
            time_err = 0.001;

        return (val) / (stamp - stamp_buffer[prev_index]) + val_buffer[prev_index];
    }

private:
    bool init;
    uint32_t index;

    T val_buffer[Size];
    T stamp_buffer[Size];
};

template <typename T, uint8_t Size>
class Linear_Interpolator
{
public:
    Linear_Interpolator(const T &x_min, const T &x_max, const T y_val[]) : x_min(x_min), x_max(x_max)
    {
        x_diff = (x_max - x_min) / (Size - 1);
        memcpy(y_buffer, y_val, sizeof(T) * Size);
        for (int i = 0; i < Size - 1; i++)
            y_diff[i] = y_buffer[i + 1] - y_buffer[i];
    }

    T getVal(const T &input) const
    {
        if (input < x_min)
            return y_buffer[0];
        else if (input > x_max)
            return y_buffer[Size - 1];
        else
        {
            uint8_t id = (input - x_min) / x_diff;
            return y_buffer[id] + y_diff[id] * (input - x_min - x_diff * id);
        }
    }

    T y_buffer[Size];
    T x_min;
    T x_max;

private:
    T y_diff[Size - 1];
    T x_diff;
};
