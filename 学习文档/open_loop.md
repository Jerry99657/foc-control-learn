# FOC 开环速度控制算法移植指南 (Arduino -> STM32 HAL)

本文档旨在介绍如何将邓哥 FOC（基于 Arduino/ESP32 的实现）的**开环速度控制算法**，无缝移植到本项目的 STM32G431 平台中。

## 1. 移植综述
在将基于 Arduino 的代码移植到 STM32 HAL 库时，核心的**数学运算**（如克拉克变换、帕克变换、SPWM/SVPWM 占空比计算）是完全一致的。主要的差异在于**底层硬件外设的驱动层**：
1. **PWM 的初始化与输出**：Arduino 使用 `ledcWrite()` 等函数，而 STM32 需要操作定时器寄存器，比如 `__HAL_TIM_SET_COMPARE()`。
2. **时间获取（微秒）**：Arduino 中使用 `micros()` 获取运行微秒数，STM32 中需要自行配置定时器或利用内核计数器（如 `DWT`）来获取精确的微秒时间。

## 2. 硬件层适配与替换

### 2.1 PWM 输出的替换
原示例代码中，向通道写入 PWM 使用：
```c
// Arduino
ledcWrite(0, dc_a * 255); // 占空比 0~1 的映射基数是 255
```

在我们的 STM32G4 工程中，使用高级定时器 **TIM1** 作为 PWM 发生器。其计数周期 (`Period`/`ARR`) 在 `tim.c` 中配置为 `16999`。所以占空比的基准映射值应改为 `16999`。
对应到 STM32 的代码如下：
```c
// STM32 (使用 TIM1 的 CH1, CH2, CH3)
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dc_a * 16999);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dc_b * 16999);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dc_c * 16999);
```

> [!TIP]
> **PWM 启动**：在 STM32 中，必须在初始化后显式启动对应通道才能正常输出：
> `HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);` 

### 2.2 获取微秒级时间 (`micros()`)
FOC 控制算法需要积分时间 (`Ts`) 来计算走过的角度。Arduino 提供现成的 `micros()`，在 STM32 中可以利用 ARM 内核的 **DWT**（Data Watchpoint and Trace）时钟周期计数器来实现非常高精度的微秒级读取：

```c
// STM32 中的 micros() 替代实现，利用内核 DWT 计数器
// 需要在初始化阶段开启：
// CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
// DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

uint32_t micros(void) {
    // 根据系统主频计算当前经过了多少微秒。比如系统频率 170MHz
    return DWT->CYCCNT / (SystemCoreClock / 1000000); 
}
```

---

## 3. 控制代码具体移植

为了保持代码的模块化，我们建议你在 `Core/Src` 新建控制相关的 `.c` 并在 `main.c` 包含，或直接把代码写在 `main.c` 的 `/* USER CODE BEGIN 0 */` 区域。

### 3.1 头文件与变量定义
首先在代码最上方引入数学库，并定义需要的全局变量。

```c
#include "math.h"
#include "tim.h"  // 包含定时器句柄 htim1

// 初始变量
float voltage_power_supply = 12.0f; // 假设系统供电电压为12V
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0, Ualpha = 0, Ubeta = 0;
float Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;

// 工具宏与常数
#define _PI 3.14159265358979323846f
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
```

### 3.2 基础数学运算函数
纯数学运算部分可以直接一字不差地复用过来：

```c
// 电角度求解
// 根据机械角度和极对数算出当前的电角度
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}

// 归一化角度到 [0, 2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2 * _PI);   
  return a >= 0 ? a : (a + 2 * _PI);  
}
```

### 3.3 设置占空比 `setPwm` (关键修改点)
在这里我们需要将原版的 `ledcWrite` 替换为 STM32 的 HAL 宏写入。

```c
// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc) {
  // 计算占空比：通过设定电压占供电电压的比例，将占空比限制从 0 到 1
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // 写入PWM到 TIM1 的通道 1, 2, 3
  // 16999 是 TIM1 配置的计数周期 Period (ARR)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dc_a * 16999));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dc_b * 16999));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc_c * 16999));
}
```

### 3.4 逆变换：帕克与克拉克 `setPhaseVoltage`
也是直接复用原版逻辑，它最后会调用上面适配好的 `setPwm` 函数。

```c
// 将给定的交轴 (Uq) 、直轴 (Ud) 电压，按指定角度变换为三相电压并输出
void setPhaseVoltage(float Uq, float Ud, float angle_el) {
  // 加上初始零点偏移并归一化
  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
  
  // 帕克逆变换 (Park Inverse)
  // 因为是开环，我们将输入的 Uq 直接解算
  Ualpha =  -Uq * sin(angle_el) + Ud * cos(angle_el); 
  Ubeta  =   Uq * cos(angle_el) + Ud * sin(angle_el); 

  // 克拉克逆变换 (Clark Inverse) 和中心对齐偏移
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta - Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha - sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  
  // 更新占空比
  setPwm(Ua, Ub, Uc);
}
```

### 3.5 开环速度逻辑核心 `velocityOpenloop`
核心循环中的时间积分是开环运行的关键，使用我们上面提及的伪 `micros()`。

```c
// 开环速度控制函数
// target_velocity: 目标角速度 (rad/s)
float velocityOpenloop(float target_velocity){
  // 获取当前的微秒数
  uint32_t now_us = micros();  
  
  // 计算当前每个Loop的运行时间间隔，单位转化为秒 (s)
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;

  // 溢出或异常处理：如果时间间隔异常，给个默认值 (1ms)
  // 这是为了防止计数器跳变时产生极端大数值导致角度暴冲
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
  
  // 通过时间间隔的积分算出目标需要运行的机械角度： V * T = S
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);

  // 设置 Uq 电压：这个值直接影响开环输出的固定力矩
  // 限制为电压的 1/3。注意：Uq 最大只能设为 voltage_power_supply/2
  float Uq = voltage_power_supply / 3;
  
  // 调用电压输出函数，极对数设为电机实际极对数（示例为7）
  setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 7));
  
  // 记录本轮时间戳供下个周期计算增量使用
  open_loop_timestamp = now_us;  

  return Uq;
}
```

---

## 4. 在主循环中整合 (main.c 的修改)

要让电机真正跑起来，你需要把上面的初始化和主函数整合到你的 `main.c` 里。对应你的工程结构，推荐如下插入：

```c
/* USER CODE BEGIN 0 */
// ... (在这里粘贴前面的所有函数与变量定义) ...
/* USER CODE END 0 */

int main(void)
{
  // ... HAL 默认初始化代码 ...
  MX_TIM1_Init();
  
  /* USER CODE BEGIN 2 */
  // 1. 开启 DWT 计数器，提供微秒级精度支持
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 2. 开启 TIM1 三个通道的 PWM 输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  
  // 如果你在 CubeMX 配置了互补通道（CH1N等），这里可能还需要启动互补输出：
  // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 调用开环速度控制，目标速度设置为 5.0 rad/s
    velocityOpenloop(5.0f);
    
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

## 总结
按照本指南将算法移植至 STM32 后，通过控制算法强行合成连续改变的旋转磁场（虚拟电角度均匀递增），以此拖拽永磁转子跟随旋转，即可实现典型的开环（强拖）控制。这种控制策略在不使用任何编码器的情况下，就能使电机完成平滑转动。
