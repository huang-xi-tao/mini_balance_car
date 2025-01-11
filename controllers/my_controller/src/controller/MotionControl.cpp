#include "robot_likeAscento.hpp"
#include "Controller.hpp"
#include <math.h>

float Controller_Class::drive(const float dt)
{
    return drive_pid.update(0, cmd.forward.d_val,
                            -pos_forward_err, (robot.legL_ptr->wheel_vel + robot.legR_ptr->wheel_vel) / 2.f, dt);
}
// lock the motor in vel loop
float Controller_Class::drive_vel_lock(const float dt)
{
    return drive_lock_pid.update(cmd.forward.d_val, -pos_forward_err,
                                 (robot.legL_ptr->wheel_vel + robot.legR_ptr->wheel_vel) / 2.f, 0, dt);
}

void Controller_Class::roll(float &left_roll /*N*/, float &right_roll /*N*/, const float dt)
{
    roll_err = set_roll - this->robot.posture_ptr->roll;
    d_roll_err = set_d_roll - this->robot.posture_ptr->_droll;

    float torque;
    roll_pid.max_int = robot.param_ptr->core.ROLL_INT_MAX / knee_output_psc;
    roll_pid.max_out = robot.param_ptr->core.ROLL_MAX / knee_output_psc;
    torque = roll_pid.update(0, 0, -roll_err, -d_roll_err, dt) * knee_output_psc;
    left_roll = -torque,
    right_roll = torque;

    // float d_err;

    // float cos = cosf(robot.posture_ptr->leg_angle),
    //       sin = sinf(robot.posture_ptr->leg_angle);

    // float t4 = (sin * 2 * (robot.posture_ptr->_quaternion.w * robot.posture_ptr->_quaternion.y - robot.posture_ptr->_quaternion.x * robot.posture_ptr->_quaternion.z) - cos * (1 - 2 * (robot.posture_ptr->_quaternion.x * robot.posture_ptr->_quaternion.x + robot.posture_ptr->_quaternion.y * robot.posture_ptr->_quaternion.y)));
    // (-robot.posture_ptr->roll + set_roll);//
    // roll_err = (-robot.posture_ptr->roll - set_roll * t4) / sqrtf(set_roll * set_roll + 1); // sine value of err angle, linearize to err angle
    // float d_roll_angle = cos * robot.posture_ptr->rotation[0] - sin * robot.posture_ptr->rotation[2];
    // bound(roll_err,0.05f);

    // d_err = roll_lpf.update(this->d_roll_err);
}
