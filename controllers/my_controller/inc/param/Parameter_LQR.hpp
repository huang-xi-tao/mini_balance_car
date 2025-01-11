#ifndef _PARAMETER_LQR_H_
#define _PARAMETER_LQR_H_

#define PARAM_LQR_LEVEL 7U

namespace Param_LQR{
typedef struct
{
    float K11[PARAM_LQR_LEVEL];
    float K12[PARAM_LQR_LEVEL];
    float K13[PARAM_LQR_LEVEL];
    float K14[PARAM_LQR_LEVEL];
    float K15[PARAM_LQR_LEVEL];
    float K16[PARAM_LQR_LEVEL];
    float K17[PARAM_LQR_LEVEL];

    float K21[PARAM_LQR_LEVEL];
    float K22[PARAM_LQR_LEVEL];    
    float K23[PARAM_LQR_LEVEL];
    float K24[PARAM_LQR_LEVEL];
    float K25[PARAM_LQR_LEVEL];
    float K26[PARAM_LQR_LEVEL];
    float K27[PARAM_LQR_LEVEL];

    float K11A[PARAM_LQR_LEVEL];
    float K12A[PARAM_LQR_LEVEL];
    float K13A[PARAM_LQR_LEVEL];
    
    float K21A[PARAM_LQR_LEVEL];
    float K22A[PARAM_LQR_LEVEL];
    float K23A[PARAM_LQR_LEVEL];
} Ground;

typedef struct
{
    float K11[PARAM_LQR_LEVEL];
    float K12[PARAM_LQR_LEVEL];
    float K13[PARAM_LQR_LEVEL];
    float K14[PARAM_LQR_LEVEL];
    float K15[PARAM_LQR_LEVEL];

    float K21[PARAM_LQR_LEVEL];
    float K22[PARAM_LQR_LEVEL];
    float K23[PARAM_LQR_LEVEL];
    float K24[PARAM_LQR_LEVEL];
    float K25[PARAM_LQR_LEVEL];
} Yaw;
};

typedef struct{
    Param_LQR::Ground gnd;
    Param_LQR::Yaw    yaw;
}Param_LQR_Set;

#endif
