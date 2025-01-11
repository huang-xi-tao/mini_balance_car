#include "leg.hpp"

Leg2::Leg2(const uint8_t config, WB6_Parameter *param) : knee(DDJ_M15_MAX_TORQUE,
                                                              DDJ_M15_BEMF_C,
                                                              DDJ_M15_TORQUE_CURRENT_PSC,
                                                              DDJ_M15_PHASE_R),
                                                         wheel(DDJ_M1505A_MAX_TORQUE,
                                                               DDJ_M15_BEMF_C,
                                                               DDJ_M15_TORQUE_CURRENT_PSC,
                                                               DDJ_M15_PHASE_R)
{
    knee_angle = angle = d_angle = 0;
    len = d_len = dd_len = 0;
    wheel_pos = wheel_vel = 0;

    this->ok = false;

    this->tilt = this->d_tilt = this->cos_tilt =
        this->round_tilt = this->tilt_rev = 0;

    this->diff_lpf.init(1000.f, 200.f);
    this->d_tilt_lpf.init(1000.f, 30.f);
    this->sin_knee = this->sin_knee_psc = 0;

    this->wheel_pos = this->_wheelPosNow = -1.24;
    param_ptr = param;
}

void Leg2::Start(void)
{
    this->force_arm = param_ptr->kin.WHEEL_DISTANCE / 2;
}

void Leg2::calLegLength()
{
    const float alpha = M_PI / 6.f;
    const float a = 0.15;
    const float b = 0.30;
    const float c = 0.08;
    const float d = 0.21;
    const float e = 0.30;

    float beta = -this->_q1;
    float phi = beta + alpha;
    float L1 = sqrt(a * a + b * b - 2 * a * b * cosf(phi));

    float val1 = (c * c + L1 * L1 - d * d) / (2 * L1 * c);
    bound(val1, 0.99f, -0.99f);
    float val2 = (b * b + L1 * L1 - a * a) / (2 * L1 * b);
    bound(val2, 0.99f, -0.99f);

    float theta1 = acosf(val1);
    float theta2 = acosf(val2);
    float theta3 = M_PI - theta1 - theta2;
    this->knee_angle = theta3;
    if (isnan(this->knee_angle))
    {
        this->knee_angle = 0;
    }
    this->len = b * sin(beta) + e * sin(theta3 - beta);
    this->d_len = -b * cos(this->_q1) * this->_dq1;
    this->dd_len = 0;
}

void Leg2::Update_Leg(const float pitch, const float d_pitch, const float *q4, const float *dq4, float dt)
{
    this->_q1 = q4[0];
    this->_q2 = q4[1];

    this->_dq1 = dq4[0];
    this->_dq2 = dq4[1];

    this->wheel_pos = -this->_wheelPosNow + this->_q2 + this->_q1; /*+ this->_q2 + pitch*/ // 轮子里程更新
    this->wheel_vel = this->_dq2 + this->_dq1; /*+ this->_dq2 + d_pitch*/                  // q3 降    q4 升

    float dpitch = 0;
    // this->knee_angle = _q1;            // in old controller, knee_angle is always positive
    // knee->angle  =   2*knee_angle
    this->sin_knee = sinf(knee_angle); // knee_angle is the knee motor's angle, each leg has one.
    float sinPsc = sin_knee;
    if (sinPsc < 0.2f && sinPsc >= 0.f)
        sinPsc = 0.2f; // why is this?
    else if (sinPsc > -0.2f && sinPsc <= 0.f)
        sinPsc = -0.2f; // why is this?
    this->sin_knee_psc = sinPsc;
    // this->angle = _q2 + this->knee_angle; // not strict, may cause problem
    // this->d_angle = _dq2 + _dq3 / 2;
    this->angle = _q1;
    this->d_angle = _dq1;
    calLegLength();
}

// void Leg2::Force_Arm_Estimate(Quaternion q, float leg_angle, float vel_forward, float vel_yaw, float roll,
//                               bool offground)
// {
//     float cos = cosf(leg_angle),
//           sin = sinf(leg_angle);

//     float t4 = (sin * 2 * (q.w * q.y - q.x * q.z) - cos * (1 - 2 * (q.x * q.x + q.y * q.y)));

//     // force arm estimation
//     float tan_theta, cos_theta;
//     if (offground)
//     {
//         tan_theta = 0;
//         cos_theta = 1;
//     }
//     else
//     {
//         float curve_roll = -(vel_forward * vel_yaw) / GRAVITY;
//         float curve_roll_err = (-roll - curve_roll * t4) / sqrtf(curve_roll * curve_roll + 1);

//         tan_theta = tanf(curve_roll_err);
//         cos_theta = 1 / sqrtf(1 + tan_theta * tan_theta);
//     }

//     float temp = (param_ptr->core.WHEEL_DISTANCE / 2 + len * tan_theta);

//     if (temp < 0.02f)
//         temp = 0.02f;

//     force_arm = temp * cos_theta;

//     load_psc = force_arm / (param_ptr->core.WHEEL_DISTANCE); // L1 = / (L1 + L2)
// }
