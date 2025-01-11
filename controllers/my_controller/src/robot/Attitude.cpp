#include "Attitude.hpp"

Attitude_Class::Attitude_Class(WB6_Parameter &param) : _quaternion{1, 0, 0, 0}
// init(false)
{
    // this->init = false;
    this->update_cnt = 0;

    this->head_pitch = this->d_head_pitch = 0;
    this->leg_angle = 0;
    this->leg_len = param.core.LEG_LEN_STRAIGHT;

    this->height = 0;
    this->vel_forward = this->vel_yaw = 0;
    this->yaw = this->d_yaw = this->yaw_rev = 0;
    param_ptr = &param;
}
// head posture
void Attitude_Class::getBodyPosture(float stamp)
{
    this->_quaternion.normalize();
    this->_quaternion.toEulerAngle(&this->_roll, &this->_pitch, &this->_yaw);
    this->_dyaw = rotation[0], this->_dpitch = rotation[1], this->_droll = rotation[2];
    float R[3][3];
    this->_quaternion.toRotationMatrix(R);
    float Rt[3][3];
    matrixTranspose(R, Rt);
#if sim_to_real
    float G[3] = {0, 0, 1.0};
#else
    float G[3] = {0, 0, 9.8};
#endif
    float RG[3] = {0};
    matrix33_multiply_vector3(Rt, G, RG);
#if sim_to_real
    this->_xaccl = (this->accl[0] - RG[0]) * 9.81;
    this->_yaccl = (this->accl[1] - RG[1]) * 9.81;
    this->_zaccl = (this->accl[2] - RG[2]) * 9.81;
#else
    this->_xaccl = this->accl[0] - RG[0];
    this->_yaccl = this->accl[1] - RG[1];
    this->_zaccl = this->accl[2] - RG[2];
#endif
    // this->_xvel = this->inter0.update(this->_xaccl ,stamp);
    // this->_yvel = this->inter1.update(this->_yaccl ,stamp);
    // this->_zvel = this->inter2.update(this->_zaccl ,stamp);
}

// void Attitude_Class::calcCrossLink(float stamp)
// {
//     const float L1 = 0.024f; // cross link length defined by machine
//     const float L2 = param_ptr->kin.HIP_LINK_LEN;
//     float gamma[2], dgamma[2], ddgamma[3];
//     gamma[0] = -this->_qMotor[2];
//     gamma[1] = -this->_qMotor[6];
//     dgamma[0] = this->dgamaL.update(gamma[0], stamp);
//     dgamma[1] = this->dgamaR.update(gamma[1], stamp);

//     // dgamma[0] = -this->_dqMotor[2];
//     // dgamma[1] = -this->_dqMotor[6];
//     for (int id = 0; id < 2; id++)
//     {
//         float biaskneeangle = 0.2717f;
//         float virtuallinklength = sqrtf(L1 * L1 + L2 * L2 + 2 * L1 * L2 * cosf(gamma[id]));
//         float alpha = asinf(L1 / virtuallinklength * sinf(gamma[id]));
//         float dalpha = L1 * cosf(gamma[id] - alpha) / (L2 * cosf(alpha) + L1 * cosf(gamma[id] - alpha)) * dgamma[id];
//         this->_qbodyEst[2 + id * 4] = -(gamma[id] - 2 * alpha) - biaskneeangle;
//         this->_dqbodyEst[2 + id * 4] = -(dgamma[id] - 2 * dalpha);
//         this->_tau3K[id] = 1 / (sinf(gamma[id]) * sinf(gamma[id] - 2 * alpha));
//         bound(_tau3K[id], 2.0f, 0.8f); // suibian shezhi de
//         // printf("tau3K[%d] = %f\n",id,_tau3K[id]);
//     }
//     this->_ddqbodyEst[2] = this->diff2.update(this->_dqbodyEst[2], stamp);
//     this->_ddqbodyEst[6] = this->diff6.update(this->_dqbodyEst[6], stamp);
// }

// void Attitude_Class::calcParallelLink(float stamp)
// {
//     const float L1 = param_ptr->kin.LITTLE_LINK_LEN; // cross link length defined by machine
//     const float L2 = param_ptr->kin.HIP_LINK_LEN;
//     this->_qbodyEst[2] = this->_qMotor[2];   // - biaskneeangle;
//     this->_qbodyEst[6] = this->_qMotor[6];   // - biaskneeangle;
//     this->_dqbodyEst[2] = this->_dqMotor[2]; // - biaskneeangle;
//     this->_dqbodyEst[6] = this->_dqMotor[6];
// }

// update
void Attitude_Class::attitude_update(float stamp)
{
    getBodyPosture(stamp);
    // calcParallelLink(stamp);
    for (size_t i = 0; i < 4; i++)
    {
        /* code */
        this->_qbodyEst[i] = this->_qMotor[i];
        this->_dqbodyEst[i] = this->_dqMotor[i];
    }
    // YHJ_TODO:这里修改了diff.update,目前传入的参数应该是dt
    this->_ddqbodyEst[0] = this->diff0.update(this->_dqbodyEst[0], stamp);
    this->_ddqbodyEst[1] = this->diff1.update(this->_dqbodyEst[1], stamp);
    this->_ddqbodyEst[2] = this->diff2.update(this->_dqbodyEst[2], stamp);
    this->_ddqbodyEst[3] = this->diff3.update(this->_dqbodyEst[3], stamp);

    // this->_ddqbodyEst[4] = this->diff4.update(this->_dqbodyEst[4], stamp);
    // this->_ddqbodyEst[5] = this->diff5.update(this->_dqbodyEst[5], stamp);
    // this->_ddqbodyEst[7] = this->diff7.update(this->_dqbodyEst[7], stamp);
}

void Attitude_Class::pitch_estimate(void)
{
    float t0 = _quaternion.w * _quaternion.y - _quaternion.z * _quaternion.x;

    float c1 = 1.f - 2.f * (_quaternion.x * _quaternion.x + _quaternion.y * _quaternion.y),
          c2 = 2.f * t0;
    float t1 = sqrtf(c1 * c1 + c2 * c2);

    float t2 = 2.f * (_quaternion.w * _quaternion.x + _quaternion.y * _quaternion.z) / c1;
    float t3 = sqrtf(1 - 4 * t0 * t0);

    head_pitch = t0 < 0 ? -acosf(c1 / t1) : acosf(c1 / t1);

    if (mabs(head_pitch) > 2 * M_PI)
    {
        head_pitch = pitch_last;
    }
    d_head_pitch = 2.f * t2 * t0 * rotation[0] + rotation[1] - t2 * t3 * rotation[2];
    // d_head_pitch = 2.f * t2 * t0 * rotation[0] + rotation[1] - t2 * t3 * rotation[2];

    if (mabs(d_head_pitch) > 100)
    {
        d_head_pitch = d_pitch_last;
    }

    pitch_last = head_pitch;
    d_pitch_last = d_head_pitch;
    // printf("attitude pitch:%.3f\n",head_pitch);
}

void Attitude_Class::update_heading(void)
{
    float cos = cosf(head_pitch),
          sin = sinf(head_pitch);

    float x = (1 - 2 * (_quaternion.y * _quaternion.y + _quaternion.z * _quaternion.z)) * cos + 2 * (_quaternion.x * _quaternion.z + _quaternion.w * _quaternion.y) * sin;
    float y = 2 * ((_quaternion.x * _quaternion.y + _quaternion.w * _quaternion.z) * cos + (_quaternion.y * _quaternion.z - _quaternion.w * _quaternion.x) * sin);
    yaw = atan2f(y, x);

    if (yaw < -M_PI / 2 && yaw > M_PI / 2)
        yaw_rev--;
    if (yaw > M_PI / 2 && yaw < -M_PI / 2)
        yaw_rev++;

    d_yaw =
        rotation[0] * 2 * (_quaternion.x * _quaternion.z - _quaternion.w * _quaternion.y) +
        rotation[1] * 2 * (_quaternion.y * _quaternion.z + _quaternion.w * _quaternion.x) +
        rotation[2] * (1 - 2 * (_quaternion.x * _quaternion.x + _quaternion.y * _quaternion.y));
}
void Attitude_Class::roll_estimate(float legL_len, float legR_len, float wheel_distance)
{
    // float _roll, _pitch, _yaw;

    float cos = cosf(leg_angle),
          sin = sinf(leg_angle);

    // float t0 = 2.f * (_quaternion.w * _quaternion.y + _quaternion.x * _quaternion.z), // 1/2s{\theta}+sin
    //     t1 = 1.f - 2.f * (_quaternion.y * _quaternion.y + _quaternion.z * _quaternion.z),
    //       t2 = 2.f * (_quaternion.w * _quaternion.x - _quaternion.y * _quaternion.z),
    //       t3 = 2.f * (_quaternion.w * _quaternion.z + _quaternion.x * _quaternion.y);

    // this->roll = -(sin * t0 - cos * t1) * (cos * t2 - sin * t3) + (cos * t0 + sin * t1) * (sin * t2 + cos * t3);
    float t4 = (sin * 2.f * (_quaternion.w * _quaternion.y - _quaternion.x * _quaternion.z) - cos * (1 - 2.f * (_quaternion.x * _quaternion.x + _quaternion.y * _quaternion.y)));
    this->roll = this->_roll;

    ground_tilt = atanf(-roll / t4) - atanf((legL_len - legR_len) / wheel_distance);

    // float d_angle = cos * rotation[0] - sin * rotation[2];
}

void Attitude_Class::wheel_estimate(const float wheel_radius, const float dt, Leg2 &legL, Leg2 &legR)
{
    vel_forward = (legL._dq2 + legR._dq2) / 2.f * wheel_radius;
    vel_yaw = d_yaw;
}

void Attitude_Class::tilt_estimate(WB6_Parameter &param, Leg2 &legL, Leg2 &legR)
{
    //     float ang0 = head_pitch + param.mass.body_ang,
    //           angl = legL.angle + head_pitch,
    //           angr = legR.angle + head_pitch;

    //     float xl = -legL.len * sinf(angl) - param.mass.body_d * cosf(ang0),
    //           xr = -legR.len * sinf(angr) - param.mass.body_d * cosf(ang0),
    //           zl = legL.len * cosf(angl) - param.mass.body_d * sinf(ang0),
    //           zr = legR.len * cosf(angr) - param.mass.body_d * sinf(ang0);

    //     legL.tilt = tilt_angle_update(atan2f(-xl, zl), legL.round_tilt, legL.tilt_rev);
    //     legR.tilt = tilt_angle_update(atan2f(-xr, zr), legR.round_tilt, legR.tilt_rev);
    //     tilt = tilt_angle_update(atan2f(-xl - xr, zl + zr), round_tilt, tilt_rev);

    //     legL.cos_tilt = zl / sqrtf(xl * xl + zl * zl);
    //     legR.cos_tilt = zr / sqrtf(xr * xr + zr * zr);
    //     cos_tilt = (zl + zr) / sqrtf((zl + zr) * (zl + zr) + (xl + xr) * (xl + xr));

    //     if (legL.cos_tilt < 0.1f)
    //         legL.cos_tilt = 0.1f;
    //     if (legR.cos_tilt < 0.1f)
    //         legR.cos_tilt = 0.1f;
    //     if (cos_tilt < 0.1f)
    //         cos_tilt = 0.1f;

    // #if sim_to_real
    //     legL.d_tilt = legL.d_tilt_lpf.update(legL.d_angle + rotation[1]);
    //     legR.d_tilt = legR.d_tilt_lpf.update(legR.d_angle + rotation[1]);
    // #else
    //     legL.d_tilt = legL.d_angle + rotation[1];
    //     legR.d_tilt = legR.d_angle + rotation[1];
    // #endif
    legL.tilt = _pitch;
    legR.tilt = _pitch;
    tilt = (legL.tilt + legR.tilt) * 0.5f;
    legL.d_tilt = _dpitch;
    legR.d_tilt = _dpitch;
    d_tilt = (legL.d_tilt + legR.d_tilt) * 0.5f;
    legL.cos_tilt = cosf(legL.tilt);
    legR.cos_tilt = cosf(legR.tilt);
    cos_tilt = (legL.cos_tilt + legR.cos_tilt) * 0.5;
}
