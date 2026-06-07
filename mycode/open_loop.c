#include "open_loop.h"

// 微秒获取函数，通过DWT计数器实现
uint32_t micros(void) {
    // 假设系统频率 170MHz
    return DWT->CYCCNT / (SystemCoreClock / 1000000); 
}

// 初始变量
float voltage_power_supply = 12.0f; // 假设系统供电电压为12V
float shaft_angle = 0;
uint32_t open_loop_timestamp = 0; // 改为 uint32_t 防止 float 精度丢失导致的时间停止
float zero_electric_angle = 0, Ualpha = 0, Ubeta = 0;
float Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;

// 工具宏与常数
#define _PI 3.14159265358979323846f
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

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

// 设置占空比 setPwm (关键修改点)
// 在这里我们需要将原版的 ledcWrite 替换为 STM32 的 HAL 宏写入。

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


// 开环速度控制函数
// target_velocity: 目标角速度 (rad/s)
float velocityOpenloop(float target_velocity){
  // 获取当前的微秒数
  uint32_t now_us = micros();  
  
  // 如果时间还没有走过1微秒（由于STM32主频极高，循环速度远快于1us），直接跳过积分
  if(now_us == open_loop_timestamp) {
    return shaft_angle;
  }
  
  // 计算当前每个Loop的运行时间间隔，单位转化为秒 (s)
  // 由于两者都是 uint32_t，相减会自动处理溢出回环问题，永远是正数的差值
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;

  // 异常处理：如果时间间隔跳变过大（大于0.5秒，比如断点调试），给个默认值防止暴冲
  if(Ts > 0.5f) Ts = 1e-3f;
  
  // 通过时间间隔的积分算出目标需要运行的机械角度： V * T = S
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts); 
  
  // 记录本次循环的时间，为下次计算准备
  open_loop_timestamp = now_us; 
  
  // 驱动函数：直接根据当前的机械角度，固定一个 Uq (比如 4V)，生成对应的三相正弦波
  // 这里为了简单起见，我们假设 Ud=0，Uq 取一个常数 4.0f (约等于 1/3 供电电压)
  float current_angle = _electricalAngle(shaft_angle, 7); // 极对数 pole_pairs 为 7
  setPhaseVoltage(4.0f, 0.0f, current_angle);

  return shaft_angle; // 返回当前的角度，方便调试
}

