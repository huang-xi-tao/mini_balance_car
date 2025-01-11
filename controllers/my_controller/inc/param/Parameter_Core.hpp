#ifndef _PARAMETER_CORE_H_
#define _PARAMETER_CORE_H_

typedef struct
{
    float FORWARD_MAX_VEL;
    float ROTATE_MAX_VEL;

    const float LEG_LEN_STRAIGHT;
    const float LEG_LEN_RETRACT;
    const float LEG_MAX_LEN;
    const float LEG_MIN_LEN;
    const float WHEEL_RADIUS;
    const float WHEEL_DISTANCE;

    float UPPER_MASS;
    float WHEEL_MASS;

    float LEG_INERTIA;
    float WHEEL_INERTIA;
    float WHEEL_FRICTION;

    float ACCL_LIMIT;
    float ANG_ACCL_LIMIT;

    float MAX_ROLL_SET;
    float MAX_ROLL;
    float MAX_ROLL_RATE;

    float MAX_PITCH;
    float MAX_PITCH_VEL;

    float MAX_HEIGHT;
    float MIN_HEIGHT;
    float MAX_HEIGHT_VEL;

    float LEG_LEN_KP;
    float LEG_LEN_KI;
    float LEG_LEN_KD;

    float DRIVE_KP;
    float DRIVE_KD;

    float LEG_TILT_KP;
    float LEG_TILT_KI;
    float LEG_TILT_KD;
    float LEG_TILT_INT_MAX;
    float LEG_TILT_OUT_MAX;

    float ROLL_KP;
    float ROLL_KI;
    float ROLL_KD;
    float ROLL_INT_MAX;
    float ROLL_MAX;

    float IMPEDANCE_KP;
    float IMPEDANCE_KD;
    float IMPEDANCE_KP_LIFT;
    float IMPEDANCE_KD_LIFT;

    float LEG_LEN_DIFF_KP;
    float LEG_LEN_DIFF_KD;

    float TRANSITION_UP_VEL;
    float TRANSITION_DOWN_VEL;

    float PITCH_PROTECT_MAX;
    float ROLL_PROTECT_MAX;

    float TAIL_UP_POS;
    float TAIL_DOWN_POS;
    float TAIL_DOWN_POS_FIRST;
    float TAIL_CHANGE_VEL;
    float TAIL_LEN;
    float TAIL_MASS;
} Param_Core_t;

#endif
