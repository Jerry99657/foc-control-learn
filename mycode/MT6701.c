#include "MT6701.h"
#include "spi.h"

uint8_t MT6701_Data[3]={0}; 
float Motor_Angle;
volatile int Angle_Raw=0;
float Encoder_Angle=0;
float Elec_Angle;
float Angle;
float Encoder_Offset = 0.0f; // 定义偏移量，移除对 Paremeter.h 的依赖
static volatile uint8_t mt6701_continuous = 0U;

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
	  Angle_Raw=(MT6701_Data[0]<<6)|(MT6701_Data[1]>>2);
		Angle = 2*PI - (2*PI*Angle_Raw/(1<<14));
		Encoder_Angle=Diff_Indentify(Angle);
		Motor_Angle=Rad2Deg(Encoder_Angle);
		Elec_Angle=((Angle-Encoder_Offset)*POLE_PAIRS);
		if (mt6701_continuous) {
			CS_Enable;
			HAL_SPI_TransmitReceive_DMA(&hspi1, MT6701_Data, MT6701_Data, 3);
		}
	}
}

void MT6701_StartContinuous(void)
{
	if (mt6701_continuous) return;
	mt6701_continuous = 1U;
	CS_Enable;
	HAL_SPI_TransmitReceive_DMA(&hspi1, MT6701_Data, MT6701_Data, 3);
}

void MT6701_StopContinuous(void)
{
	mt6701_continuous = 0U;
	HAL_SPI_DMAStop(&hspi1);
	CS_Disable;
}

uint8_t MT6701_IsContinuous(void)
{
	return mt6701_continuous;
}
