#ifndef KINEMATIC_HPP
#define KINEMATIC_HPP

#include "A1_def.hpp"
#include <webots/robot.h>
#include <webots/supervisor.h>
#define PRINT_LOG 0

double getWholebodyMass();

void getRelativePositon(const char *def, double *pos);

void getGlobalPositon(const char *def, double *pos);

void getRobotGlobalPositon(double *pos);

void getRobotGlobalOrientation(double *ori);

void getRobotGlobalEularAngle(double *eular);

void getRobotGlobalWholeBodyComPos(double *com);

void getRobotRelativeWholeBodyComPos(double *relative_com);
// without wheel mass
void getRobotRelativeUpperBodyComPos(double *relative_upper_com);

void initDevices();

const double *getAccleration();

const double *getGyro();

const double *getInertialUnit();

void getMotorPosition(double *position);

void setMotorPosition(double *position);
void setTailMotorPosition(double *position);

void getMotorTorque(double *torque);

void setMotorTorque(double *torque_set, unsigned int mode);

void setMotorTorqueConstraint(float &torque_set, const float rpm_now, const float torque_limit, const float rpm_limit);

#endif
