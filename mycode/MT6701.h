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

extern volatile float Motor_Angle;
extern volatile int Angle_Raw;
extern volatile float Encoder_Angle;
extern volatile float Elec_Angle;
extern volatile float Angle;
extern float Encoder_Offset;
extern uint8_t MT6701_Data[3];

// StartContinuous 仅使能定频采样；实际DMA请求由10kHz ADC回调中的
// MT6701_Service() 发起，不再在DMA完成回调里无限自触发。
void MT6701_StartContinuous(void);
void MT6701_StopContinuous(void);
void MT6701_Service(void);
uint8_t MT6701_IsContinuous(void);
uint8_t MT6701_IsHealthy(void);
uint16_t MT6701_GetRawAngle(void);
uint8_t MT6701_GetStatus(void);
uint32_t MT6701_GetSampleCounter(void);
uint32_t MT6701_GetErrorCounter(void);
uint16_t MT6701_GetSampleAgeCycles(void);

#endif
