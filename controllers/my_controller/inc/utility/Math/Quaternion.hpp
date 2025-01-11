#pragma once
#include "MathFunc.hpp"

/**
 * @brief quaternion computation library 
 */
class Quaternion
{
public:
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(const float w, const float x, const float y, const float z)
        : w(w), x(x), y(y), z(z)
    {}
    Quaternion(const float w[3], const float dt);
    Quaternion(const float euler_angle[3]);
    Quaternion(const float rotation_matrix[3][3]);

    float norm(void);
    void normalize(void);
    void set(float w, float x, float y, float z);

    Quaternion derivative(const float w[3]) const
    {
        return Quaternion(
            0.5 * (w[0] * -this->x + w[1] * -this->y + w[2] * -this->z),
            0.5 * (w[0] *  this->w + w[1] * -this->z + w[2] *  this->y),
            0.5 * (w[0] *  this->z + w[1] *  this->w + w[2] * -this->x),
            0.5 * (w[0] * -this->y + w[1] *  this->x + w[2] *  this->w));
    }

    Quaternion &operator+=(const Quaternion &delta)
    {
        w += delta.w;
        x += delta.x;
        y += delta.y;
        z += delta.z;
        return *this;
    }

    Quaternion inverse(void) const
    {
        return Quaternion(this->w, -this->x, -this->y, -this->z);
    }

    // TODO migrate this to operator * to increase efficiency
    Quaternion &operator*=(const Quaternion &delta)
    {
        float x, y, z, w;

        x = this->x * delta.w + this->y * delta.z - this->z * delta.y +
            this->w * delta.x;
        y = -this->x * delta.z + this->y * delta.w + this->z * delta.x +
            this->w * delta.y;
        z = this->x * delta.y - this->y * delta.x + this->z * delta.w +
            this->w * delta.z;
        w = -this->x * delta.x - this->y * delta.y - this->z * delta.z +
            this->w * delta.w;

        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;

        return *this;
    }

    Quaternion &operator*=(const float scalar)
    {
        this->w *= scalar;
        this->x *= scalar;
        this->y *= scalar;
        this->z *= scalar;
        return *this;
    }

    Quaternion &operator/=(const float scalar)
    {
        this->w /= scalar;
        this->x /= scalar;
        this->y /= scalar;
        this->z /= scalar;
        return *this;
    }

    void toEulerAngle(volatile float *roll, volatile float *pitch,
                      volatile float *yaw) const;

    void toRotationMatrix(float R[3][3])
    {
        R[0][0] = 1 - 2*(y*y + z*z); R[0][1] = 2*(x*y - w*z);     R[0][2] = 2*(x*z + w*y);
        R[1][0] = 2*(x*y + w*z);     R[1][1] = 1 - 2*(x*x + z*z); R[1][2] = 2*(y*z - w*x);
        R[2][0] = 2*(x*z - w*y);     R[2][1] = 2*(y*z + w*x);     R[2][2] = 1 - 2*(x*x + y*y);
    }

    void rotate_vector3(const float input[3], float output[3])
    {
        float R[3][3];
        this->toRotationMatrix(R);
        matrix33_multiply_vector3(R, input, output);
    }   
};

Quaternion operator+(const Quaternion &lhs, const Quaternion &rhs);
Quaternion operator*(const Quaternion &lhs, const Quaternion &rhs);
Quaternion operator*(const float scalar, const Quaternion &rhs);
Quaternion operator*(const Quaternion &lhs, const float scalar);
Quaternion operator/(const Quaternion &lhs, const float scalar);
