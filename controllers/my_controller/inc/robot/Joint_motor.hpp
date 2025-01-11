#pragma once

#include <stdint.h>
#include "Parameter.hpp"
/**
 * @brief abstract motor class
 */
class Joint_motor
{
public:
    Joint_motor(const float max_torque = DDJ_M15_MAX_TORQUE,
                const float bemf = DDJ_M15_BEMF_C,
                const float torque = DDJ_M15_TORQUE_CURRENT_PSC,
                const float resistance = DDJ_M15_PHASE_R) : max_output_torque(max_torque), bemf_constant(bemf), torque_constant(torque)
    {
        this->phase_resistance = resistance;
        this->mode_set = CTRL_VOLTAGE;
        this->dir = 1;
        this->mode = CTRL_TORQUE; // CTRL_MODE_INVALID;

        this->torque_set = this->vq_set = this->iq_set = 0;
        this->round_angle = 0;
        this->max_torque = max_output_torque;
        this->min_torque = -max_output_torque;
        this->max_vel = 0;
        this->rev = this->update_cnt = 0;

        this->init = false;
        this->status = OJBK;
        this->release_brake();
    }
    enum dir_t
    {
        DIR_CW = 1,
        DIR_CCW = -1
    };

    enum ctrl_mode_t
    {
        CTRL_DUTY_CYCLE = 0,
        CTRL_VOLTAGE = 1,
        CTRL_CURRENT = 2,
        CTRL_TORQUE = 3,
        CTRL_VELOCITY = 4,
        CTRL_POSITION = 5,
        CTRL_MODE_INVALID = -1
    };

    enum brake_cmd_t
    {
        NO_CMD = 0,
        SET = 1,
        RELEASE = 2
    };

    void set_brake()
    {
        this->brake_cmd = SET;
    }

    void release_brake()
    {
        this->brake_cmd = RELEASE;
    }

    void setVoltage(const float voltage);

    void setCurrent(const float current);

    void setTorque(const float torque);

    void setPosition(const float pos);

    void update(const float angle_vel, const float Vin);

    void update(const float angle, const float angle_vel, const float iq, const float V_in);

    void update(const float angle, const float angle_vel, const float torque);

protected:
    Joint_motor(const float max_output_torque,
                const float torque_speed_const,
                const float bemf) : max_output_torque(max_output_torque), bemf_constant(bemf), torque_constant(1.0f)
    {
        this->phase_resistance = bemf / torque_speed_const;

        this->mode_set = CTRL_TORQUE;
        this->dir = DIR_CW;
        this->mode = CTRL_TORQUE;

        this->torque_set = this->vq_set = this->iq_set = 0;
        this->round_angle = 0;
        this->max_torque = max_output_torque;
        this->min_torque = -max_output_torque;
        this->rev = this->update_cnt = 0;

        this->init = false;
        this->status = DISCONNECT;
        this->release_brake();
    }

public:
    enum
    {
        OJBK = 0,
        ERROR = 1,
        SEVERE_WARNING = 2,
        WARNING = 4,
        DISCONNECT = 8
    };

    bool ojbk(void)
    {
        return !status;
    }

    bool init;
    uint8_t status;
    uint8_t check_connect_flag;

    const float max_output_torque;
    const float bemf_constant;
    const float torque_constant;
    float phase_resistance;

    ctrl_mode_t mode_set;
    int8_t dir;
    ctrl_mode_t mode;
    brake_cmd_t brake_cmd;

    float torque_set;
    float angle_set;
    float vq_set;
    float iq_set;

    float round_angle;
    int rev;

    float torque;
    float angle;
    float angle_vel;
    float iq;
    float temp;

    float max_torque;
    float min_torque;
    float max_vel;

    // TODO: UPDATE PROTOCOL OR NOT
    uint16_t enc_pos;
    uint16_t enc_vel;
    uint16_t enc_iq;

    uint32_t update_cnt;
};