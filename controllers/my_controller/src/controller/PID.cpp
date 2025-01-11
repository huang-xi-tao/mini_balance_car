#include "PID.hpp"
#include "MathFunc.hpp"

float PID_Controller::update(const float target, const float input, const float dt)
{
    float output, err = target - input;
    if (!isfinite(err))
        return 0; // TODO CATCH BUG

    output = this->kp * err;
    if (this->ki)
    {
        this->err_int += this->ki * err * dt;
        bound((this->err_int), this->max_int);
        output += this->err_int;
    }

    bound(output, this->max_out);
    this->output = output;
    return output;
}

float PID_Controller::update(const float target, const float d_target,
                             const float input, const float d_input, const float dt)
{
    float output,
        err = target - input,
        d_err = d_target - d_input;
    if (!isfinite(err) || !isfinite(d_err))
        return 0; // TODO CATCH BUG

    output = this->kp * err + this->kd * d_err;
    if (this->ki)
    {
        this->err_int += this->ki * err * dt;
        bound((this->err_int), this->max_int);
        output += this->err_int;
    }

    bound(output, this->max_out);
    this->output = output;
    return output;
}

// PID with err thresh hold
float PID_Controller::update(const float target, const float d_target,
                             const float input, const float d_input, const float int_thresh_err,
                             const float dt = 0)
{
    float output,
        err = target - input,
        d_err = d_target - d_input;
    if (!isfinite(err) || !isfinite(d_err))
        return 0; // TODO CATCH BUG

    output = this->kp * err + this->kd * d_err;
    if (this->ki && fabs(err) > int_thresh_err)
    {
        this->err_int += this->ki * err * dt;
        bound((this->err_int), this->max_int);
        output += this->err_int;
    }
    else
    {
        this->err_int = 0;
    }
    bound(output, this->max_out);
    this->output = output;
    return output;
}

float PID_Controller::update(const float target, const float d_target,
                             const float input, const float d_input,
                             const float Max_int, const float Min_int,
                             const float Max_out, const float Min_out,
                             const float dt)
{
    float output,
        err = target - input,
        d_err = d_target - d_input;
    if (!isfinite(err) || !isfinite(d_err))
        return 0; // TODO CATCH BUG

    output = this->kp * err + this->kd * d_err;
    if (this->ki)
    {
        this->err_int += this->ki * err * dt;
        bound((this->err_int), Max_int, Min_int);
        output += this->err_int;
    }

    bound(output, Max_out, Min_out);
    this->output = output;
    return output;
}

float PID_Controller::updatePD(const float qdesire, const float dqdesire,
                               const float qest, const float dqest)
{
    return this->kp * (qdesire - qest) + this->kd * (dqdesire - dqest);
}
