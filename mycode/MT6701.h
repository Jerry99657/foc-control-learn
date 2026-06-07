#ifndef __MT6701_H_
#define __MT6701_H_

#include "spi.h"
#include "gpio.h"
#include "math.h"
#include "tim.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define POLE_PAIRS 7

#define Deg2Rad(deg) ((deg)*PI/180.0f)
#define Rad2Deg(rad) ((rad)*180.0f/PI)

#define CS_Enable HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port,MT6701_CSN_Pin,GPIO_PIN_RESET)
#define CS_Disable HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port,MT6701_CSN_Pin,GPIO_PIN_SET)

float Diff_Indentify(float Diff);

extern float Motor_Angle;
extern volatile int Angle_Raw;
extern float Encoder_Angle;
extern float Elec_Angle;
extern float Angle;
extern float Encoder_Offset;
extern uint8_t MT6701_Data[3];

#endif
