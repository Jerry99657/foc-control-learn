#ifndef __CLOSE_LOOP_H
#define __CLOSE_LOOP_H

#include "main.h"
#include "tim.h"
#include "math.h"
#include "MT6701.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

void setPwm_CL(float Ua, float Ub, float Uc);
void setPhaseVoltage_CL(float Uq, float Ud, float angle_el);
void alignSensor(void);
void positionCloseLoop(float target_angle_rad);

#endif
