#include "JerryFOC.h"

// ================= 参数与全局变量 =================
static float voltage_power_supply = 12.0f; 
static float zero_electric_angle = 0;
static int DIR = 1; 

// PWM 占空比
static float dc_a = 0, dc_b = 0, dc_c = 0;

// 多圈角度跟踪变量
static float prev_Angle = 0;
static float full_rotations = 0;
static float mechanical_zero_offset = 0;

// 速度与力矩目标
float JerryFOC_Target_Velocity = 0.0f;
float JerryFOC_Target_Uq = 0.0f; // 由速度环 PID 计算得出，供给内层 FOC

// 把滤波常数从 0.01 改为 0.002 (2ms)，大幅降低反馈延迟！
LowPassFilter M0_Vel_Flt = { .Tf = 0.002f, .y_prev = 0.0f }; 

// 【PID 调参关键区】
// P（比例）：决定了对误差的响应力度。如果电机软绵绵没力气，就加大 P；如果电机发出高频尖锐的震动声，就减小 P。
// I（积分）：消除稳态误差。过大的 I 会导致电机缓慢震荡（打嗝式的转动）。如果抖动且转得慢，建议把 I 设为 0 或者极小值！
PIDController vel_loop_M0 = { .P = 0.05f, .I = 0.2f, .D = 0.0f, .output_ramp = 100000.0f, .limit = 6.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f };


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
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

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


// ================= 速度控制接口 =================
void JerryFOC_setVelocity(float target_vel) {
    // 设置全局目标速度，供 TIM2 中断调用
    JerryFOC_Target_Velocity = target_vel;
}

static float filtered_velocity_global = 0.0f;

float JerryFOC_getVelocity(void) {
    return filtered_velocity_global;
}

// 主循环极速执行函数：只负责根据最新的 Uq 和电角度执行底层换相
void JerryFOC_run(void) {
    JerryFOC_setPhaseVoltage(JerryFOC_Target_Uq, 0.0f, JerryFOC_getElectricalAngle());
}

// ================= TIM2 1ms 定时器中断回调 =================
// 所有的速度计算、低通滤波、PID 运算均在此处以 1ms 的严格时序定频执行
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // 1. 获取最新角度并追踪多圈
        static float last_mechanical_angle = 0;
        float current_mechanical_angle = JerryFOC_getAngle(); 
        
        // 2. 裸速度计算 (delta S / 0.001s)
        float raw_velocity = (current_mechanical_angle - last_mechanical_angle) / 0.001f;
        last_mechanical_angle = current_mechanical_angle;
        
        // 3. 速度低通滤波
        filtered_velocity_global = JerryFOC_LPF_Calc(&M0_Vel_Flt, DIR * raw_velocity);
        
        // 4. 计算速度误差
        float error = JerryFOC_Target_Velocity - filtered_velocity_global;
        
        // 5. PID 计算得出目标 Uq 电压
        JerryFOC_Target_Uq = JerryFOC_PID_Calc(&vel_loop_M0, error);
    }
}
