#include "close_loop.h"

// 为了避免与 open_loop.c 冲突，我们在本文件使用静态变量定义供电电压等
static float cl_voltage_power_supply = 12.0f; 

// 闭环控制需要的全局变量
float cl_zero_electric_angle = 0;
int DIR = 1; 
static float dc_a = 0, dc_b = 0, dc_c = 0;

// 多圈角度跟踪变量
static float prev_Angle = 0;
static float full_rotations = 0;
static float mechanical_zero_offset = 0;

// 约束宏
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// 归一化角度到 [0, 2PI]
static float _normalizeAngle_CL(float angle){
    float a = fmod(angle, 2 * PI);   
    return a >= 0 ? a : (a + 2 * PI);  
}

// 闭环电角度计算
static float _electricalAngle_CL() {
    // 根据 MT6701 读取到的机械角度 Angle 计算电角度，并减去零点偏移
    return _normalizeAngle_CL((DIR * POLE_PAIRS) * Angle - cl_zero_electric_angle);
}

// 设置PWM占空比
void setPwm_CL(float Ua, float Ub, float Uc) {
    // 限制占空比从 0 到 1
    dc_a = _constrain(Ua / cl_voltage_power_supply, 0.0f , 1.0f );
    dc_b = _constrain(Ub / cl_voltage_power_supply, 0.0f , 1.0f );
    dc_c = _constrain(Uc / cl_voltage_power_supply, 0.0f , 1.0f );

    // 写入PWM到 TIM1 的通道 1, 2, 3
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dc_a * 16999));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dc_b * 16999));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc_c * 16999));
}

// 帕克与克拉克逆变换
void setPhaseVoltage_CL(float Uq, float Ud, float angle_el) {
    angle_el = _normalizeAngle_CL(angle_el);
    
    // 帕克逆变换
    float Ualpha = -Uq * sin(angle_el) + Ud * cos(angle_el); 
    float Ubeta  =  Uq * cos(angle_el) + Ud * sin(angle_el); 

    // 克拉克逆变换与偏移
    float Ua = Ualpha + cl_voltage_power_supply/2;
    float Ub = (sqrt(3)*Ubeta - Ualpha)/2 + cl_voltage_power_supply/2;
    float Uc = (-Ualpha - sqrt(3)*Ubeta)/2 + cl_voltage_power_supply/2;
    
    setPwm_CL(Ua, Ub, Uc);
}

// 传感器校准与零点标定
void alignSensor(void) {
    // 强制输出 Uq=3V, 电角度=270度，让电机强行对齐到固定磁场
    setPhaseVoltage_CL(3.0f, 0.0f, 3.0f * PI / 2.0f);
    
    // 延时等待电机稳定锁死
    HAL_Delay(3000); 
    
    // 记录此时的传感器电角度作为零点偏移
    cl_zero_electric_angle = (DIR * POLE_PAIRS) * Angle;
    
    // 重置多圈跟踪变量，并将此时的机械位置定义为目标绝对零点
    prev_Angle = Angle;
    full_rotations = 0;
    mechanical_zero_offset = Angle;
    
    // 关闭输出
    setPhaseVoltage_CL(0.0f, 0.0f, 3.0f * PI / 2.0f);
}

// 闭环位置控制器 (P 控制)
void positionCloseLoop(float target_angle_rad) {
    float current_Angle = Angle;
    float d_angle = current_Angle - prev_Angle;
    
    // 跨越 2PI 的跳变检测 (假设采样率足够高，单次步进不会超过 PI)
    if(d_angle > PI) {
        full_rotations -= 1.0f; // 从 0 跳到 2PI，相当于反转了一圈
    } else if(d_angle < -PI) {
        full_rotations += 1.0f; // 从 2PI 跳到 0，相当于正转了一圈
    }
    prev_Angle = current_Angle;
    
    // 真实的连续累加机械角度 (包含多圈)，并以标定位置作为 0 度参考点
    float Sensor_Angle = full_rotations * 2 * PI + current_Angle - mechanical_zero_offset; 
    
    float Kp = 0.133f; // P 控制器增益参数
    
    // 计算位置误差，输出目标力矩 Uq 电压
    float Uq = Kp * (target_angle_rad - DIR * Sensor_Angle) * 180.0f / PI;
    
    // 电压输出限幅
    Uq = _constrain(Uq, -6.0f, 6.0f); 
    
    // 执行电压输出，角度传入实时测算的电角度
    setPhaseVoltage_CL(Uq, 0.0f, _electricalAngle_CL());
}
