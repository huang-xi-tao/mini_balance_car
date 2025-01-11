#ifndef _PARAM_DIABLO_H_
#define _PARAM_DIABLO_H_

#include "Parameter.hpp"

#define DDJ_M15_MAX_TORQUE 100.f
#define DDJ_M15_BEMF_C 0.85f
#define DDJ_M15_PHASE_R 0.70f
#define DDJ_M15_TORQUE_CURRENT_PSC 1.4

#define DDJ_M1505A_MAX_TORQUE 100.f

#define DDJ_M15_SET_BRAKE 0xAA
#define DDJ_M15_RELEASE_BRAKE 0x55

#define CAN_M15_VQ_CMD_PSC (32768 / 28.f) // 100.f
#define CAN_M15_IQ_CMD_PSC (32767 / 110.0f)
static const Param_Core_t param_core =
    {
        .FORWARD_MAX_VEL = 6.f,
        .ROTATE_MAX_VEL = 2.f,

        .LEG_LEN_STRAIGHT = 0.50f,
        .LEG_LEN_RETRACT = 0.25f,
        .LEG_MAX_LEN = 0.40f,
        .LEG_MIN_LEN = 0.25f,
        .WHEEL_RADIUS = 0.12f,
        .WHEEL_DISTANCE = 0.60f,

        .UPPER_MASS = 30.f,
        .WHEEL_MASS = 4.f,

        .LEG_INERTIA = 0.021f,
        .WHEEL_INERTIA = 0.0066f,
        .WHEEL_FRICTION = 0.3f,

        .ACCL_LIMIT = 2.f,
        .ANG_ACCL_LIMIT = 5.f,

        .MAX_ROLL_SET = 0.1f,
        .MAX_ROLL = 0.1f,
        .MAX_ROLL_RATE = 0.6f,

        .MAX_PITCH = 0.8f, // 0.3f*M_PI,
        .MAX_PITCH_VEL = 2.f,

        .MAX_HEIGHT = 0.40f,
        .MIN_HEIGHT = 0.25f,
        .MAX_HEIGHT_VEL = 0.5f,

        .LEG_LEN_KP = 50.f,
        .LEG_LEN_KI = 200.f,
        .LEG_LEN_KD = 0.4f,

        .DRIVE_KP = 10.f,
        .DRIVE_KD = 0.8f,

        .LEG_TILT_KP = 20.f,
        .LEG_TILT_KI = 40.f,
        .LEG_TILT_KD = 0.9f,
        .LEG_TILT_INT_MAX = 3.f,
        .LEG_TILT_OUT_MAX = 8.f,

        .ROLL_KP = 3000.f,
        .ROLL_KI = 50.f,
        .ROLL_KD = 1.0f,
        .ROLL_INT_MAX = 100.0f,
        .ROLL_MAX = 1000.f,

        .IMPEDANCE_KP = 36.f,
        .IMPEDANCE_KD = 3.6f,
        .IMPEDANCE_KP_LIFT = 18.f,
        .IMPEDANCE_KD_LIFT = 2.0f,

        .LEG_LEN_DIFF_KP = 800.f,
        .LEG_LEN_DIFF_KD = 40.f,

        .TRANSITION_UP_VEL = 0.2f, // 0.6f,
        .TRANSITION_DOWN_VEL = 0.2f,

        .PITCH_PROTECT_MAX = 2.0,
        .ROLL_PROTECT_MAX = 0.8f,

        .TAIL_UP_POS = 0.0,
        .TAIL_DOWN_POS = -2.8,
        .TAIL_DOWN_POS_FIRST = -2.8,
        .TAIL_CHANGE_VEL = 1.0,
        .TAIL_LEN = 0.32,
        .TAIL_MASS = 5,
};

static const Param_Kin_t kin_param =
    {
        .BODY_LINK_LEN = 0.0945f,
        .KNEE_LINK_LEN = 0.140f,
        .LITTLE_LINK_LEN = 0.09f,
        .WHEEL_LINK_LEN = 0.0502f,
        .WHEEL_DISTANCE = 0.490f,
        .MAX_LEG_LEN = 0.27f,
        .MIN_LEG_LEN = 0.03f,
        .WHEEL_RADIUS = 0.0945f,
        .LEG_LEN_MAX = 0.27f,
        .LEG_LEN_MIN = 0.03f,
        .LEG_LEN_INIT_STAND = 0.0120f,
};

static const InitMotorPosition _initMotorPos =
    {

};

static const Param_LQR_Set param_lqr =
    {
        .gnd =
            {
                .K11 = {-3.2607, -3.6773, -2.4341, -1.7711, -1.3701, -1.1049, -0.91821},
                //    .K12 = {-5.14,-4.961,-4.9194,-4.5817,-3.7333,-3.5935,-2.836},//wheel integral
                .K12 = {-12.14, -10.961, -8.9194, -7.5817, -6.7333, -5.5935, -4.836}, // wheel integral
                .K13 = {-0.71429, -0.41054, -0.39592, -0.32504, -0.27685, -0.24186, -0.21521},
                //.K14 = {-7.217,-10.462,-13.552,-16.469,-19.221,-21.828,-24.312},
                .K14 = {-25.826, -25.693, -25.328, -34.704, -40.832, -40.743, -45.469},
                .K15 = {-4, -4, -4, -4.5, -5, -5.5, -6},
                //        .K15 = {-0.8606,-1.0,-1.5,-2.0,-2.5,-3.0,-3.5},
                .K16 = {-10.1716, -8.4475, -8.5153, -8.5398, -8.5518, -8.559, -8.5639},
                .K17 = {-10.3324, -7.7173, -7.7853, -8.0116, -8.2819, -8.5613, -8.838},

                .K21 = {5.274, 5.783, 5.919, 5.966, 5.986, 5.996, 5.001},   // hip angle P
                .K22 = {25.41, 25.7, 25.6, 25.86, 25.96, 26.06, 26.26},     // hip integral I
                .K23 = {0.8, 1.0104, 1.029, 1.0364, 1.0399, 1.0418, 1.043}, // hip diff D
                .K24 = {-2.5308, -1.5536, -1.1008, -1.1008, -1.1008, -0.60461, -0.52812},
                .K25 = {-0.2196, -0.19211, -0.17261, -0.15917, -0.14884, -0.1404, -0.13326},
                .K26 = {-2.9093, -1.4455, -0.84507, -0.56058, -0.40309, -0.30599, -0.24148},
                .K27 = {-2.7639, -1.1802, -0.66651, -0.44153, -0.32082, -0.24724, -0.19842},

                .K11A = {-1.218, -1.1559, -1.0686, -0.97239, -0.87922, -0.79491, -0.72114},
                .K12A = {-60.812, -65.183, -69.959, -74.166, -77.548, -80.161, -82.155},
                .K13A = {-2.4215, -2.7654, -3.2438, -3.8052, -4.4183, -5.0645, -5.7329},

                .K21A = {1.5808, 1.6465, 1.7292, 1.8087, 1.8756, 1.9285, 1.9693},
                .K22A = {-65.514, -64.022, -60.524, -55.837, -50.896, -46.24, -42.077},
                .K23A = {-2.204, -2.2054, -2.1658, -2.0998, -2.0281, -1.9621, -1.9055}},
        .yaw =
            {
                .K11 = {6.0243, 5.6205, 5.1783, 4.7368, 4.309, 3.9018, 3.519},
                .K12 = {1.8435, 1.7465, 1.6383, 1.528, 1.4187, 1.3121, 1.2094},
                .K13 = {-10.0998, -11.354, -15.226, -17.96, -20.141, -22.005, -23.667},
                .K14 = {-30.173, -38.255, -50.64, -58.718, -64.555, -68.994, -72.458},
                .K15 = {-1.69122, -1.5217, -2.0333, -2.3881, -2.6631, -2.8902, -3.0854},

                .K21 = {-7.185, -8.3662, -9.4306, -10.317, -11.044, -11.637, -12.119},
                .K22 = {-2.1826, -2.5296, -2.8516, -3.1329, -3.3772, -3.5896, -3.775},
                .K23 = {-30.813, -30.222, -36.414, -39.136, -40.624, -41.657, -42.544},
                .K24 = {-83.404, -88.386, -100.24, -99.49, -94.18, -87.18, -79.659},
                .K25 = {-3.5282, -3.1325, -3.5469, -3.5211, -3.3477, -3.13, -2.909}}};

#endif