#include "JerryFOC.h"
#include <stdint.h>
#include <stdio.h>
#include "adc.h"
#include "usart.h"

// ================= 算法实现：DWT 微秒级延时 =================
static void delay_us(uint32_t us) {
    uint32_t ticks = us * 170; // 170MHz 主频下 1us = 170 ticks
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks);
}

// ================= 大疆无人机启动音效 =================
void JerryFOC_playStartupSound(void) {
    // 经典的音效: Do-Mi-So-Do (C5, E5, G5, C6)
    uint16_t notes[] = {523, 659, 783, 1046}; 
    uint16_t durations[] = {150, 150, 150, 300}; // 持续时间 ms
    
    for (int i = 0; i < 4; i++) {
        uint32_t start_time = HAL_GetTick();
        uint32_t period_us = 1000000 / notes[i];
        uint32_t half_period_us = period_us / 2;
        
        while ((HAL_GetTick() - start_time) < durations[i]) {
            // 在 Ud 轴注入高频交变电压 (Uq=0，不会产生转矩旋转)
            JerryFOC_setPhaseVoltage(0.0f, 2.0f, 0.0f);
            delay_us(half_period_us);
            
            JerryFOC_setPhaseVoltage(0.0f, -2.0f, 0.0f);
            delay_us(half_period_us);
        }
        
        // 音符之间稍作静音停顿
        JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
        HAL_Delay(20);
    }
}

// ================= 参数与全局变量 =================
static float voltage_power_supply = 12.0f;  // 实际母线电压 12V
static float zero_electric_angle = 0;
static int DIR = 1; 

// ================= 电流环相关硬件常数 =================
#define SHUNT_RESISTOR 0.01f
#define AMP_GAIN 50.0f
#define VOLTS_TO_AMPS (1.0f / SHUNT_RESISTOR / AMP_GAIN) // 2.0f
#define ADC_OFFSET 1.65f
#define ADC_TO_VOLTS (3.3f / 4095.0f)

// ================= 模式控制变量 =================
static JerryFOC_ControlMode current_control_mode = JERRYFOC_MODE_TORQUE;

// ================= 多圈角度跟踪变量 =================
static float prev_Angle = 0;
static float full_rotations = 0;
static float mechanical_zero_offset = 0;

// 速度外环变量
float filtered_velocity_global = 0.0f;
float JerryFOC_Target_Velocity = 0.0f;
float JerryFOC_Target_Iq = 0.0f; // 由速度环 PID 计算得出，供给内层电流 FOC

// 电流内环变量
float phase_current_a = 0.0f;
float phase_current_b = 0.0f;
float iq_current = 0.0f;

// 速度低通滤波器 (Tf=0.02s, 截止频率约 8Hz，充分平滑速度噪声)
LowPassFilter M0_Vel_Flt = { .Tf = 0.02f, .y_prev = 0.0f }; 
// 电流低通滤波器 (Tf=0.001s, 截止频率约 160Hz)
LowPassFilter M0_Curr_Flt = { .Tf = 0.001f, .y_prev = 0.0f };

// 速度外环 PID (直接输出电压 Uq！限幅 2.0V 足够让云台电机达到最高速)
PIDController vel_loop_M0 = { .P = 0.05f, .I = 0.5f, .D = 0.0f, .output_ramp = 0.0f, .limit = 2.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f };
// 电流内环 PID (暂时弃用)
PIDController curr_loop_M0 = { .P = 0.0f, .I = 0.0f, .D = 0.0f, .output_ramp = 100000.0f, .limit = 6.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f };



// ================= 工具函数 =================
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

static float _normalizeAngle(float angle){
    float a = fmod(angle, 2 * PI);   
    return a >= 0 ? a : (a + 2 * PI);  
}

// ================= 算法实现：低通滤波 (1ms 定步长) =================
float JerryFOC_LPF_Calc(LowPassFilter* filter, float x) {
    // 固定的 1ms 步长，摒弃 millis() 或 micros() 的复杂调用
    float Ts = 0.001f; 
    float alpha = filter->Tf / (filter->Tf + Ts);
    float y = alpha * filter->y_prev + (1.0f - alpha) * x;
    filter->y_prev = y;
    return y;
}

// ================= 算法实现：PID 控制 (1ms 定步长) =================
float JerryFOC_PID_Calc(PIDController* pid, float error) {
    float Ts = 0.001f; // 固定的 1ms 步长
    
    // P 环
    float proportional = pid->P * error;
    
    // I 环 (Tustin 散点积分)
    float integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
    integral = _constrain(integral, -pid->limit, pid->limit);
    
    // D 环
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    // 累加
    float output = proportional + integral + derivative;
    output = _constrain(output, -pid->limit, pid->limit);

    // 变化率限幅 (输出 Ramp)
    if(pid->output_ramp > 0){
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * Ts;
    }
    
    // 保存状态供下次使用
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    
    return output;
}

// ================= 底层 FOC 换相逻辑 =================
static void setPwm(float Ua, float Ub, float Uc) {
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dc_a * 16999));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dc_b * 16999));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc_c * 16999));
}

void JerryFOC_setPhaseVoltage(float Uq, float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el);
    
    float Ualpha = -Uq * sin(angle_el) + Ud * cos(angle_el); 
    float Ubeta  =  Uq * cos(angle_el) + Ud * sin(angle_el); 

    float Ua = Ualpha + voltage_power_supply/2;
    float Ub = (sqrt(3)*Ubeta - Ualpha)/2 + voltage_power_supply/2;
    float Uc = (-Ualpha - sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
    
    setPwm(Ua, Ub, Uc);
}

// ================= 传感器标定与状态获取 =================
void JerryFOC_alignSensor(void) {
    JerryFOC_setPhaseVoltage(3.0f, 0.0f, 3.0f * PI / 2.0f);
    HAL_Delay(3000); 
    
    zero_electric_angle = (DIR * POLE_PAIRS) * Angle;
    
    prev_Angle = Angle;
    full_rotations = 0;
    mechanical_zero_offset = Angle;
    
    JerryFOC_setPhaseVoltage(0.0f, 0.0f, 3.0f * PI / 2.0f);
}

static float JerryFOC_getElectricalAngle() {
    return _normalizeAngle((DIR * POLE_PAIRS) * Angle - zero_electric_angle);
}

float JerryFOC_getAngle() {
    float current_Angle = Angle;
    float d_angle = current_Angle - prev_Angle;
    
    if(d_angle > PI) {
        full_rotations -= 1.0f; 
    } else if(d_angle < -PI) {
        full_rotations += 1.0f; 
    }
    prev_Angle = current_Angle;
    
    return full_rotations * 2 * PI + current_Angle - mechanical_zero_offset; 
}


// ================= 闭环：模式、速度、扭矩设定与获取 =================
void JerryFOC_setMode(JerryFOC_ControlMode mode) { current_control_mode = mode; }
void JerryFOC_setVelocity(float target_vel) { JerryFOC_Target_Velocity = target_vel; }
void JerryFOC_setCurrent(float target_Iq) { JerryFOC_Target_Iq = target_Iq; }
float JerryFOC_getVelocity(void) { return filtered_velocity_global; }
float JerryFOC_getPhaseCurrent_A(void) { return phase_current_a; }
float JerryFOC_getPhaseCurrent_B(void) { return phase_current_b; }
float JerryFOC_getIq(void) { return iq_current; }

// ================= Clarke 与 Park 变换 =================
void JerryFOC_Clarke(float Ia, float Ib, float* Ialpha, float* Ibeta) {
    *Ialpha = Ia;
    *Ibeta = (1.0f / sqrtf(3.0f)) * Ia + (2.0f / sqrtf(3.0f)) * Ib;
}

float JerryFOC_Park(float Ialpha, float Ibeta, float angle_el) {
    float ct = cosf(angle_el);
    float st = sinf(angle_el);
    // 这里采用 DengFOC 的 Iq 计算公式
    return Ibeta * ct - Ialpha * st;
}

// 主循环极速执行函数：只负责根据最新的 Uq 和电角度执行底层换相
void JerryFOC_run(void) {
    // 检查 ADC 注入组是否完成了一次新的转换 (由 TIM1 5kHz TRGO 自动触发)
    // 只有当新的 ADC 数据到来时，我们才执行一轮电流环，保证严格的 5kHz 同步
    if (ADC1->ISR & ADC_ISR_JEOS) {
        // 清除标志位
        ADC1->ISR |= ADC_ISR_JEOS;
        
        // 1. 获取最新角度
        float electrical_angle = JerryFOC_getElectricalAngle();

        // 2. 读取 ADC (完全避开 analogRead 的损耗，耗时 0 周期)
        uint32_t adc1_val = ADC1->JDR1;
        uint32_t adc2_val = ADC2->JDR1;

        // 3. 计算相电流
        phase_current_a = (adc1_val * ADC_TO_VOLTS - ADC_OFFSET) * VOLTS_TO_AMPS;
        phase_current_b = (adc2_val * ADC_TO_VOLTS - ADC_OFFSET) * VOLTS_TO_AMPS;

        // 4. Clarke & Park
        float Ialpha, Ibeta;
        JerryFOC_Clarke(phase_current_a, phase_current_b, &Ialpha, &Ibeta);
        float raw_Iq = JerryFOC_Park(Ialpha, Ibeta, electrical_angle);

        // 5. 电流滤波 (由于在此执行频率严格等于 5kHz，因此 Ts 固定为 0.0002s)
        float Ts_curr = 0.0002f;
        float alpha = M0_Curr_Flt.Tf / (M0_Curr_Flt.Tf + Ts_curr);
        iq_current = alpha * M0_Curr_Flt.y_prev + (1.0f - alpha) * raw_Iq;
        M0_Curr_Flt.y_prev = iq_current;

        float Uq = 0.0f;
        if (current_control_mode == JERRYFOC_MODE_VELOCITY) {
            // 速度模式下，速度环直接输出电压 Uq (忽略电流环，避免 ADC 噪声干扰)
            Uq = JerryFOC_Target_Iq; 
        } else {
            // 扭矩模式，走电流环
            float error = JerryFOC_Target_Iq - iq_current;
            float proportional = curr_loop_M0.P * error;
            float integral = curr_loop_M0.integral_prev + curr_loop_M0.I * Ts_curr * 0.5f * (error + curr_loop_M0.error_prev);
            integral = _constrain(integral, -curr_loop_M0.limit, curr_loop_M0.limit);
            curr_loop_M0.integral_prev = integral;
            curr_loop_M0.error_prev = error;
            Uq = proportional + integral;
            Uq = _constrain(Uq, -curr_loop_M0.limit, curr_loop_M0.limit);
        }

        // 7. FOC 换相
        JerryFOC_setPhaseVoltage(Uq, 0.0f, electrical_angle);
    }
}

// ================= TIM2 1ms 定时器中断回调 =================
// 所有的速度计算、低通滤波、PID 运算均在此处以 1ms 的严格时序定频执行！
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // 1. 获取最新角度并追踪多圈
        static float last_mechanical_angle = 0;
        float current_mechanical_angle = JerryFOC_getAngle(); 
        
        // 2. 裸速度计算 (delta S / 0.001s)
        float raw_velocity = (current_mechanical_angle - last_mechanical_angle) / 0.001f;
        last_mechanical_angle = current_mechanical_angle;
        
        // 3. 速度低通滤波 (恢复为正向。之前反向是因为畸变的电流环导致的假象)
        filtered_velocity_global = JerryFOC_LPF_Calc(&M0_Vel_Flt, raw_velocity);
        
        // 4. 计算速度误差
        float error = JerryFOC_Target_Velocity - filtered_velocity_global;
        
        // 5. PID 计算得出目标 Iq 电流
        if (current_control_mode == JERRYFOC_MODE_VELOCITY) {
            JerryFOC_Target_Iq = JerryFOC_PID_Calc(&vel_loop_M0, error);
        }

        // 6. 每 100ms 通过串口打印一次真实计算出的速度，让我们看看它到底读到了什么！
        static int print_counter = 0;
        if (++print_counter >= 100) {
            print_counter = 0;
            char buf[80];
            int len = snprintf(buf, sizeof(buf), "T:%.1f A:%.1f Uq:%.3f\r\n", JerryFOC_Target_Velocity, filtered_velocity_global, JerryFOC_Target_Iq);
            HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 10);
        }
    }
}
