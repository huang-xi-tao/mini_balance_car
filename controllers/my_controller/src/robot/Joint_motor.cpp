#include "MathFunc.hpp"
#include "Joint_motor.hpp"

void Joint_motor::setVoltage(const float voltage)
{
    this->vq_set = voltage;
}

void Joint_motor::setCurrent(const float current)
{
    this->iq_set = current;

    if (mode != CTRL_CURRENT)
        this->setVoltage(current * phase_resistance + angle_vel * bemf_constant);
}

void Joint_motor::setTorque(const float torque)
{
    float torque_set = torque;
    bound(torque_set, this->max_torque, this->min_torque);

    this->torque_set = torque_set;

    if (mode != CTRL_TORQUE)
        this->setCurrent(this->torque_set * torque_constant);
}

void Joint_motor::setPosition(const float pos)
{
    this->angle_set = pos;
}
// maximum output update
void Joint_motor::update(const float angle_vel, const float V_in) // Can try to change into current loop for a better control in leg
{
    // this->max_vel = V_in / bemf_constant;

    // this->max_torque = (V_in - bemf_constant * angle_vel) / phase_resistance / torque_constant; // calculate the actual maximum output  based on the motor model.
    // if (this->max_torque > this->max_output_torque)
    //     this->max_torque = this->max_output_torque;

    // this->min_torque = (-V_in - bemf_constant * angle_vel) / phase_resistance / torque_constant;
    // if (this->min_torque < -this->max_output_torque)
    //     this->min_torque = -this->max_output_torque;
}
// actual status output
void Joint_motor::update(const float angle, const float angle_vel, const float iq, const float V_in)
{
    this->iq = iq;
    this->angle_vel = angle_vel;
    this->torque = iq / torque_constant;

    if (angle - this->round_angle < -M_PI)
        this->rev++;
    else if (angle - this->round_angle > M_PI)
        this->rev--;
    this->round_angle = angle;
    this->angle = this->rev * 2 * M_PI + this->round_angle;

    this->update(angle_vel, V_in); // update the maximum output value.
}

// actual status output
void Joint_motor::update(const float angle, const float angle_vel, const float torque)
{
    this->iq = iq;
    this->angle_vel = angle_vel;
    this->torque = torque;

    if (angle - this->round_angle < -M_PI)
        this->rev++;
    else if (angle - this->round_angle > M_PI)
        this->rev--;
    this->angle = this->rev * 2 * M_PI + angle;

    this->round_angle = angle;
    // this->angle = angle;
}
