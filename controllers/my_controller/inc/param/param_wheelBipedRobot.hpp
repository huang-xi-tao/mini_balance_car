#pragma once
#define sim_to_real 0
typedef struct
{
    const float BODY_LINK_LEN;
    const float KNEE_LINK_LEN;
    const float LITTLE_LINK_LEN;
    const float WHEEL_LINK_LEN;
    const float WHEEL_DISTANCE;
    const float MAX_LEG_LEN;
    const float MIN_LEG_LEN;
    const float WHEEL_RADIUS;
    const float LEG_LEN_MAX;
    const float LEG_LEN_MIN;
    const float LEG_LEN_INIT_STAND;

} Param_Kin_t;

typedef struct
{
    const float MASS_BODY = 14.0f;
    const float MASS_KNEE = 0.0f;
    const float MASS_WHEEL = 3.0f;

    const float COM_BODY[3] = {0.004f, 0.00f, 0.004f};
    const float COM_KNEE_L[3] = {0.004f, 0.00f, 0.004f};
    const float COM_WHEEL_L[3] = {0.004f, 0.00f, 0.004f};
    const float COM_KNEE_R[3] = {0.004f, 0.00f, 0.004f};
    const float COM_WHEEL_R[3] = {0.004f, 0.00f, 0.004f};

    const float INERTIA_BODY[3][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const float INERTIA_KNEE_L[3][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const float INERTIA_WHEEL_L[3][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const float INERTIA_KNEE_R[3][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const float INERTIA_WHEEL_R[3][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    const float BODY_MASS_COM_X = 0.00f;
    const float BODY_MASS_COM_Z = 0.00f;
} DynamicParam;

typedef struct
{
#if sim_to_real
    const float KNEE_LEFT = 0.0f;
    const float KNEE_RIGHT = 0.0f;
#else
    const float KNEE_LEFT = 0.0f;
    const float KNEE_RIGHT = 0.0f;
#endif
    const float WHEEL_LEFT = 0.0f;
    const float WHEEL_RIGHT = 0.0f;
} InitMotorPosition;

typedef struct
{
    // left motor
    const float MOTOR_KP[8] = {100.0f, 100.0f, 50.0f, 100.0f, 100.0f, 100.0f, 50.0f, 100.0f};
    const float MOTOR_KI[8] = {00.0f, 00.0f, 00.0f, 00.0f, 00.0f, 00.0f, 00.0f, 00.0f};
    const float MOTOR_KD[8] = {0.50f, 0.50f, 0.3f, 0.8f, 0.5f, 0.5f, 0.3f, 0.8f};

    const float KNEE_MOTOR_KP_L = 20.0f;
    const float KNEE_MOTOR_KI_L = 0.0f;
    const float KNEE_MOTOR_KD_L = 0.8f;

    const float WHEEL_MOTOR_KP_L = 20.0f;
    const float WHEEL_MOTOR_KI_L = 0.0f;
    const float WHEEL_MOTOR_KD_L = 0.8f;

    // right motor
    const float KNEE_MOTOR_KP_R = 20.0f;
    const float KNEE_MOTOR_KI_R = 0.0f;
    const float KNEE_MOTOR_KD_R = 0.8f;

    const float WHEEL_MOTOR_KP_R = 20.0f;
    const float WHEEL_MOTOR_KI_R = 0.0f;
    const float WHEEL_MOTOR_KD_R = 0.8f;
} MotorPIDParam;
