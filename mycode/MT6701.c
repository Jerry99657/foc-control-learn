#include "MT6701.h"
#include "spi.h"
#include <stdint.h>

uint8_t MT6701_Data[3]={0}; 
volatile float Motor_Angle;
volatile int Angle_Raw=0;
volatile float Encoder_Angle=0;
volatile float Elec_Angle;
volatile float Angle;
float Encoder_Offset = 0.0f; // 定义偏移量，移除对 Paremeter.h 的依赖
static volatile uint8_t mt6701_continuous = 0U;
static volatile uint8_t mt6701_transfer_active = 0U;
static volatile uint8_t mt6701_sample_valid = 0U;
static volatile uint8_t mt6701_status = 0U;
static volatile uint16_t mt6701_sample_age_cycles = UINT16_MAX;
static volatile uint32_t mt6701_sample_counter = 0U;
static volatile uint32_t mt6701_error_counter = 0U;

// 10kHz调度下允许连续3个控制周期没有新样本；超过后有感FOC撤掉输出。
#define MT6701_MAX_SAMPLE_AGE_CYCLES 3U

// MT6701 SSI帧：14位角度 + 4位状态 + 6位CRC。
// CRC覆盖前18位，生成多项式x^6+x+1（去掉最高项后为0x03），MSB先入。
static uint8_t MT6701_CalculateCrc6(uint32_t data18)
{
	uint8_t crc = 0U;
	for (int8_t bit_index = 17; bit_index >= 0; bit_index--) {
		uint8_t feedback = (uint8_t)(((data18 >> bit_index) & 1U) ^
									 ((crc >> 5) & 1U));
		crc = (uint8_t)((crc << 1) & 0x3FU);
		if (feedback) {
			crc ^= 0x03U;
		}
	}
	return crc;
}

static uint8_t MT6701_RequestSample(void)
{
	if (!mt6701_continuous || mt6701_transfer_active) {
		return 0U;
	}

	mt6701_transfer_active = 1U;
	CS_Enable;
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
		&hspi1, MT6701_Data, MT6701_Data, 3);
	if (status != HAL_OK) {
		CS_Disable;
		mt6701_transfer_active = 0U;
		mt6701_error_counter++;
		return 0U;
	}
	return 1U;
}

float Diff_Indentify(float Diff)
{
	if(Diff<-PI)
	{
	  Diff+=2*PI;
	}
	else if(Diff>PI)
	{
	  Diff-=2*PI;
	} 
	return Diff;
}

// SPI DMA 收发完成回调函数
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance==SPI1)
	{
	  CS_Disable;
	  mt6701_transfer_active = 0U;

	  uint32_t raw_frame = ((uint32_t)MT6701_Data[0] << 16) |
						 ((uint32_t)MT6701_Data[1] << 8) |
						 (uint32_t)MT6701_Data[2];
	  uint16_t raw_angle = (uint16_t)(raw_frame >> 10);
	  uint8_t status = (uint8_t)((raw_frame >> 6) & 0x0FU);
	  uint8_t received_crc = (uint8_t)(raw_frame & 0x3FU);
	  uint8_t calculated_crc = MT6701_CalculateCrc6(raw_frame >> 6);
	  mt6701_status = status;

	  // Mg[1:0]非零表示磁场过强/过弱，Mg[3]表示失跟踪；Mg[2]
	  // 是按压状态，不影响转子角度。全0/全1帧用于识别MISO卡死。
	  if (raw_frame == 0U || raw_frame == 0xFFFFFFU ||
		  received_crc != calculated_crc || (status & 0x0BU) != 0U) {
		  mt6701_error_counter++;
		  return;
	  }

	  Angle_Raw = (int)raw_angle;
		Angle = 2*PI - (2*PI*Angle_Raw/(1<<14));
		Encoder_Angle=Diff_Indentify(Angle);
		Motor_Angle=Rad2Deg(Encoder_Angle);
		Elec_Angle=((Angle-Encoder_Offset)*POLE_PAIRS);
		mt6701_sample_age_cycles = 0U;
		mt6701_sample_valid = 1U;
		mt6701_sample_counter++;
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1) {
		CS_Disable;
		mt6701_transfer_active = 0U;
		mt6701_error_counter++;
	}
}

void MT6701_StartContinuous(void)
{
	if (mt6701_continuous) return;
	mt6701_continuous = 1U;
	mt6701_transfer_active = 0U;
	mt6701_sample_valid = 0U;
	mt6701_sample_age_cycles = UINT16_MAX;
	MT6701_RequestSample();
}

void MT6701_StopContinuous(void)
{
	mt6701_continuous = 0U;
	HAL_SPI_DMAStop(&hspi1);
	CS_Disable;
	mt6701_transfer_active = 0U;
	mt6701_sample_valid = 0U;
	mt6701_sample_age_cycles = UINT16_MAX;
}

uint8_t MT6701_IsContinuous(void)
{
	return mt6701_continuous;
}

void MT6701_Service(void)
{
	if (!mt6701_continuous) return;

	if (mt6701_sample_valid && mt6701_sample_age_cycles < UINT16_MAX) {
		mt6701_sample_age_cycles++;
	}
	MT6701_RequestSample();
}

uint8_t MT6701_IsHealthy(void)
{
	return (mt6701_continuous && mt6701_sample_valid &&
			mt6701_sample_age_cycles <= MT6701_MAX_SAMPLE_AGE_CYCLES) ? 1U : 0U;
}

uint16_t MT6701_GetRawAngle(void)
{
	return (uint16_t)Angle_Raw;
}

uint8_t MT6701_GetStatus(void)
{
	return mt6701_status;
}

uint32_t MT6701_GetSampleCounter(void)
{
	return mt6701_sample_counter;
}

uint32_t MT6701_GetErrorCounter(void)
{
	return mt6701_error_counter;
}

uint16_t MT6701_GetSampleAgeCycles(void)
{
	return mt6701_sample_age_cycles;
}
