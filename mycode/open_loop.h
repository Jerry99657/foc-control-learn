#ifndef __OPEN_LOOP_H
#define __OPEN_LOOP_H

#include "main.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>

void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
float velocityOpenloop(float target_velocity);
uint32_t micros(void);

#endif /* __OPEN_LOOP_H */
