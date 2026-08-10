/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "JerryFOC.h"
#include "MT6701.h"
#include "angle_src_encoder.h"
#include "angle_src_smo.h"
#include "startup.h"
#include <stdlib.h> // for atof
#include <string.h> // for strncmp
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_byte;
char rx_buffer[32];
uint8_t rx_index = 0;
volatile uint8_t g_need_align = 0;
volatile uint8_t g_is_sensorless = 0; // 上电默认使用C2804编码器有感模式

// 开环诊断模式全局变量（定义在 JerryFOC.c）
extern volatile uint8_t g_openloop_debug;
extern volatile float g_openloop_angle;
extern volatile float JerryFOC_Target_Velocity;  // 当前目标速度

// 无感启动配置
static StartupConfig startup_cfg = {
    .align_current = 0.40f,
    .align_duration = 0.7f,
    .align_angle = 0.0f,
    .openloop_Iq = 0.40f,
    .openloop_ramp = 10.0f,
    .switch_speed = 15.0f,
    .switch_duration = 0.6f,
    .lock_duration = 0.05f,
    .switch_timeout = 5.0f,
    .lock_loss_timeout = 0.10f,
    .lock_speed_tolerance = 6.0f,
    .lock_phase_tolerance = 1.2f,
};
static AngleSource *g_smo_src = NULL;  // 保存 SMO 角度源，切换时复用

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        if(rx_byte == '\n') {
            rx_buffer[rx_index] = '\0';
            
            if (strncmp(rx_buffer, "Speed:", 6) == 0) {
                float target = atof(rx_buffer + 6);
                // 有感速度环和编码器反馈均支持有符号速度，负目标必须
                // 原样传入；只有尚未扩展反向启动的无感模式按停止处理。
                if (g_is_sensorless && target < 0.0f) target = 0.0f;
                uint32_t primask = __get_PRIMASK();
                __disable_irq();

                if (target == 0.0f && g_is_sensorless) {
                    // 纯无感在零速时没有可观测的反电动势。Speed:0 应直接
                    // 退出启动/闭环并撤掉转矩，不能重新触发启动状态机。
                    JerryFOC_setVelocity(0.0f);
                    JerryFOC_setCurrent(0.0f);
                    JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
                    Startup_Stop();
                } else {
                    // 只有非零无感速度命令才启动状态机；有感模式直接闭环。
                    StartupState startup_state = Startup_GetState();
                    if (g_is_sensorless &&
                        (startup_state == STARTUP_IDLE || startup_state == STARTUP_FAULT)) {
                        Startup_Init(7, &startup_cfg, 10000.0f);
                        Startup_Begin();
                    }
                    JerryFOC_setMode(JERRYFOC_MODE_VELOCITY);
                    JerryFOC_setVelocity(target);
                }
                if (!primask) __enable_irq();
            } else if (strncmp(rx_buffer, "Angle:", 6) == 0) {
                float target = atof(rx_buffer + 6);
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                JerryFOC_setMode(JERRYFOC_MODE_POSITION);
                JerryFOC_setPosition(target, 0.0f);
                if (!primask) __enable_irq();
            } else if (strncmp(rx_buffer, "Torque:", 7) == 0) {
                float target = atof(rx_buffer + 7);
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
                JerryFOC_setCurrent(target);
                if (!primask) __enable_irq();
            } else if (strncmp(rx_buffer, "Motor:", 6) == 0) {
                int id = atoi(rx_buffer + 6);
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                g_is_sensorless = 0; // 切换为有感
                g_openloop_debug = 0;
                JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
                JerryFOC_setVelocity(0.0f);
                JerryFOC_setCurrent(0.0f);
                Startup_Stop();
                MT6701_StartContinuous();
                JerryFOC_selectMotor((JerryFOC_MotorID)id);
                JerryFOC_setAngleSource(AngleSrc_Encoder_Init(7));
                if (id == JERRYFOC_MOTOR_C2804) {
                    g_need_align = 1;
                }
                if (!primask) __enable_irq();
            } else if (strcmp(rx_buffer, "Sensorless") == 0) {
                // 只负责配置/待机；随后 Speed 命令启动 ALIGN->OPENLOOP 流程。
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                MT6701_StopContinuous();
                g_is_sensorless = 1;
                g_openloop_debug = 0;
                JerryFOC_selectMotor(JERRYFOC_MOTOR_C2208);
                JerryFOC_useCurrentLoop(1);
                JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
                JerryFOC_setVelocity(15.0f);
                JerryFOC_setCurrent(0.0f);
                g_openloop_angle = 0.0f;
                g_smo_src = AngleSrc_SMO_Create(&MOTOR_C2208, 10000.0f);
                JerryFOC_setAngleSource(g_smo_src);
                Startup_Init(7, &startup_cfg, 10000.0f);
                if (!primask) __enable_irq();
            } else if (strcmp(rx_buffer, "Lock") == 0) {
                // 手动命令只能请求安全切换，不能绕过反电势/速度/相位判据。
                if (g_is_sensorless) {
                    uint32_t primask = __get_PRIMASK();
                    __disable_irq();
                    g_openloop_debug = 0;
                    JerryFOC_useCurrentLoop(1);
                    if (g_smo_src) JerryFOC_setAngleSource(g_smo_src);
                    StartupState startup_state = Startup_GetState();
                    if (startup_state == STARTUP_IDLE || startup_state == STARTUP_FAULT) {
                        Startup_Begin();
                    }
                    Startup_ForceClosed();
                    JerryFOC_setMode(JERRYFOC_MODE_VELOCITY);
                    if (!primask) __enable_irq();
                }
            } else if (strncmp(rx_buffer, "OpenLoop:", 9) == 0) {
                // 纯开环诊断：固定电压 + 旋转角度
                // 格式：OpenLoop:速度,电压  例如 OpenLoop:30,6
                float speed = 10.0f, voltage = 4.0f;
                char *comma = strchr(rx_buffer + 9, ',');
                if (comma) {
                    *comma = '\0';
                    speed = atof(rx_buffer + 9);
                    voltage = atof(comma + 1);
                } else {
                    speed = atof(rx_buffer + 9);
                }
                if (speed < 2.0f) speed = 10.0f;
                if (voltage < 0.5f) voltage = 0.5f;
                if (voltage > 11.0f) voltage = 11.0f;
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                g_is_sensorless = 0;
                Startup_Stop();
                MT6701_StartContinuous();
                JerryFOC_selectMotor(JERRYFOC_MOTOR_C2208);
                JerryFOC_useCurrentLoop(0);
                JerryFOC_setAngleSource(AngleSrc_Encoder_Init(7));
                JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
                JerryFOC_setVelocity(speed);
                JerryFOC_setCurrent(voltage);
                g_openloop_debug = 1;
                g_openloop_angle = 0.0f;
                if (!primask) __enable_irq();
            }
            
            rx_index = 0;
        } else if (rx_byte != '\r') {
            if(rx_index < 31) {
                rx_buffer[rx_index++] = rx_byte;
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USB_Device_Init();
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 1. 开启 DWT 计数器，提供微秒级精度支持
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 2. 开启 TIM1 三个通道的 PWM 输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // 3. 上电默认进入C2804有感模式，开启由ADC节拍调度的MT6701采样。
  MT6701_StartContinuous();

  // 4. 开启 USART1 串口中断接收
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  HAL_UART_Transmit(&huart1, (uint8_t*)"TEST\r\n", 6, 100);

  // 5. ADC 上电自校准
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  // ===== 6. 初始化角度源：默认使用C2804的MT6701编码器 =====
  AngleSource *encoder_src = AngleSrc_Encoder_Init(MOTOR_C2804.pole_pairs);
  g_smo_src = AngleSrc_SMO_Create(&MOTOR_C2208, 10000.0f);
  JerryFOC_setAngleSource(encoder_src);

  // ===== 7. 默认电机：C2804有感电压模式 =====
  JerryFOC_selectMotor(JERRYFOC_MOTOR_C2804);
  JerryFOC_useCurrentLoop(0);

  // ===== 8. 播放大疆启动音效 =====
  JerryFOC_playStartupSound();

  // ===== 9. 零转矩待机并请求C2804编码器零点标定 =====
  JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
  JerryFOC_setCurrent(0.0f);  // 上电不输出力矩
  Startup_Init(7, &startup_cfg, 10000.0f);
  g_need_align = 1U;

  // 10. 开启 ADC 注入组：从机先启动，主机转换完成中断驱动 10kHz FOC。
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  // 开启 TIM2 定时器中断（1ms 速度/位置外环）
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_need_align) {
        g_need_align = 0;
        // 先停掉所有控制，设为空载
        JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
        JerryFOC_setCurrent(0.0f);
        // 使用编码器角度源，并调用阻塞式对齐函数
        JerryFOC_setAngleSource(AngleSrc_Encoder_Init(JerryFOC_getMotorParams()->pole_pairs));
        JerryFOC_alignSensor();
    }

    // 10kHz FOC 已由 ADC 注入完成中断驱动；主循环只处理低优先级任务。
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
