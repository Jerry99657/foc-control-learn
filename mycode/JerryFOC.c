#include "JerryFOC.h"
#include <stdint.h>
#include <stdio.h>
#include "adc.h"
#include "usart.h"
#include "angle_src_encoder.h"   // 编码器角度源 + Align
#include "angle_src_smo.h"       // SMO 角度源
#include "MT6701.h"              // 有感采样调度与健康状态
#include "smo.h"                 // SMO_Update 接口
#include "startup.h"             // 无感启动状态机
#include "arm_math.h"            // 引入 ARM CMSIS-DSP 库

// 定义放入 CCMRAM 极速执行的宏
#ifndef __RAM_FUNC
#define __RAM_FUNC __attribute__((section(".RamFunc")))
#endif

// ================= FOC 控制环路耗时测试 =================
volatile uint32_t foc_loop_cycles = 0;   // 消耗的 CPU 时钟周期
volatile float foc_loop_time_ns = 0.0f;  // 实际耗时 (纳秒)
volatile float dpwm_time_ns = 0.0f;      // DPWM 调制算法专属耗时 (纳秒)

// ================= 算法实现：DWT 微秒级延时 =================
static void delay_us(uint32_t us) {
    uint32_t ticks = us * 170; // 170MHz 主频下 1us = 170 ticks
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks);
}

// ================= 大疆无人机启动音效 =================
// ================= 反向相序适配 =================
// 注意：SWAP_MOTOR_PHASES 必须保持为 0！
// 将其设为 1 会导致 β 轴电流/电压同时反号，使 SMO PLL 误差信号
// 从 sin(θ-θ̂) 变为 sin(θ+θ̂)，以 2 倍频振荡且均值为零，PLL 永远无法锁定。
// 如果电机方向相反，请物理交换任意两根电机线，不要用软件交换。
#define SWAP_MOTOR_PHASES 0

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
static const MotorParams *active_motor = &MOTOR_C2804;  // 上电默认 C2804
static AngleSource *angle_source = NULL;                 // 角度源（编码器或 SMO）

// 纯开环诊断模式
volatile uint8_t g_openloop_debug = 0;
volatile float g_openloop_angle = 0.0f;
static float g_shared_prev_Uq = 0.0f;  // 开环/闭环共享的上拍电压
static float g_shared_prev_Ud = 0.0f;
static float g_shared_prev_Ualpha = 0.0f;
static float g_shared_prev_Ubeta = 0.0f;
static volatile uint8_t foc_fast_loop_enabled = 1U;

// C2804有感速度指令规划：反转时先减速到零，再沿相反方向加速。
#define SENSORED_ACCEL_LIMIT 100.0f   // rad/s^2
#define SENSORED_DECEL_LIMIT 150.0f   // rad/s^2
static float sensored_velocity_reference = 0.0f;

// ================= 电流环相关硬件常数 =================
#define SHUNT_RESISTOR 0.01f
#define AMP_GAIN 50.0f
#define VOLTS_TO_AMPS (1.0f / SHUNT_RESISTOR / AMP_GAIN) // 2.0f
#define ADC_TO_VOLTS (3.3f / 4095.0f)
#define CURRENT_OFFSET_SAMPLES 2000U

static float adc_offset_count_a = 2047.5f;
static float adc_offset_count_b = 2047.5f;
static uint32_t adc_offset_accum_a = 0U;
static uint32_t adc_offset_accum_b = 0U;
static uint32_t adc_offset_samples = 0U;

// ================= 模式控制变量 =================
static volatile JerryFOC_ControlMode current_control_mode = JERRYFOC_MODE_TORQUE;

// 电流环使能标志 (默认 0：电压控制模式)
volatile uint8_t flag_use_current_loop = 0;

// ================= 角度源与电机切换 =================
void JerryFOC_setAngleSource(AngleSource *src) {
    angle_source = src;
}

AngleSource* JerryFOC_getAngleSource(void) {
    return angle_source;
}

void JerryFOC_selectMotor(JerryFOC_MotorID id) {
    if (id == JERRYFOC_MOTOR_C2208) {
        active_motor = &MOTOR_C2208;
        flag_use_current_loop = 1;  // C2208 高阻电机必须用电流环
    } else {
        active_motor = &MOTOR_C2804;
        flag_use_current_loop = 0;  // C2804 低阻电机可用电压模式
    }

    // 重载电流环 PID
    id_loop_M0.P = iq_loop_M0.P = active_motor->cur_P;
    id_loop_M0.I = iq_loop_M0.I = active_motor->cur_I;
    id_loop_M0.limit = iq_loop_M0.limit = active_motor->cur_limit;
    id_loop_M0.output_ramp = iq_loop_M0.output_ramp = active_motor->cur_ramp;

    // 重载速度环 PID
    vel_loop_M0.P = active_motor->vel_P;
    vel_loop_M0.I = active_motor->vel_I;
    vel_loop_M0.limit = active_motor->vel_limit;
    vel_loop_M0.output_ramp = active_motor->vel_ramp;
    M0_Vel_Flt.Tf = active_motor->vel_filter_Tf;

    // 重载位置环 PID
    pos_loop_M0.P = active_motor->pos_P;
    pos_loop_M0.limit = active_motor->pos_limit;
    pos_loop_M0.output_ramp = active_motor->pos_ramp;

    // 重置所有 PID 积分和误差状态
    id_loop_M0.integral_prev = iq_loop_M0.integral_prev = 0;
    id_loop_M0.error_prev = iq_loop_M0.error_prev = 0;
    id_loop_M0.output_prev = iq_loop_M0.output_prev = 0;
    vel_loop_M0.integral_prev = vel_loop_M0.error_prev = vel_loop_M0.output_prev = 0;
    pos_loop_M0.integral_prev = pos_loop_M0.error_prev = pos_loop_M0.output_prev = 0;
    M0_Vel_Flt.y_prev = 0;
    sensored_velocity_reference = 0.0f;
}

const MotorParams* JerryFOC_getMotorParams(void) {
    return active_motor;
}

// 速度外环变量
volatile float filtered_velocity_global = 0.0f;
volatile float JerryFOC_Target_Position = 0.0f;
volatile float JerryFOC_Target_Velocity = 0.0f;
volatile float JerryFOC_Target_Iq = 0.0f; // 由速度环 PID 计算得出，供给内层电流 FOC
float velocity_limit_global = 30.0f; // 默认速度限幅 30 rad/s

void JerryFOC_useCurrentLoop(uint8_t enable) {
    flag_use_current_loop = enable;
}


// 电流内环变量
volatile float phase_current_a = 0.0f;
volatile float phase_current_b = 0.0f;
volatile float id_current = 0.0f;
volatile float iq_current = 0.0f;

// 速度低通滤波器 (Tf=0.02s, 截止频率约 8Hz，充分平滑速度噪声)
LowPassFilter M0_Vel_Flt = { .Tf = 0.02f, .y_prev = 0.0f }; 
// 电流低通滤波器 (Tf=0.0001s, 截止频率约 1600Hz，极小化相位延迟)
LowPassFilter M0_Curr_Flt = { .Tf = 0.0001f, .y_prev = 0.0f };

// 位置外环 PID (纯P控制即可，P=2.0 比较柔和，限幅默认为 30.0 rad/s)
PIDController pos_loop_M0 = { .P = 2.0f, .I = 0.0f, .D = 0.0f, .output_ramp = 0.0f, .limit = 30.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f };

// 速度内环 PID (直接输出电压 Uq！限幅设为 SVPWM 线性区最大相电压 Udc/sqrt(3) ≈ 6.93V)
PIDController vel_loop_M0 = { .P = 0.05f, .I = 0.5f, .D = 0.0f, .output_ramp = 0.0f, .limit = 6.93f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f };

// 电流最内环 PID (带宽降低为 250rad/s，避免高频震荡)
// P = Ls * 250 = 0.00086 * 250 = 0.215
// I = Rs * 250 = 2.3 * 250 = 575
PIDController id_loop_M0 = { .P = 0.215f, .I = 575.0f, .D = 0.0f, .output_ramp = 100000.0f, .limit = 12.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f };
PIDController iq_loop_M0 = { .P = 0.215f, .I = 575.0f, .D = 0.0f, .output_ramp = 100000.0f, .limit = 12.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f };

void JerryFOC_BumplessTransition(void) {
    // 兼容旧 API：仅请求状态机执行安全切换。真正的 PI 无扰初始化
    // 在 ADC/FOC 中断内、使用新角度投影后的实际电流完成。
    Startup_ForceClosed();
}



// ================= 工具函数 =================
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

__RAM_FUNC static float _normalizeAngle(float angle){
    // 采用极速的 浮点->整型(VCVT) 汇编指令求余，彻底抛弃几百个周期的 fmod
    // 用乘以 1/2PI 的方式替代除法，FPU 单精度乘法仅需 1 个周期
    float a = angle - (int)(angle * 0.159154943091895f) * (2.0f * PI);
    return a >= 0.0f ? a : (a + 2.0f * PI);
}

// ================= 算法实现：低通滤波 (1ms 定步长) =================
__RAM_FUNC float JerryFOC_LPF_Calc(LowPassFilter* filter, float x) {
    // 固定的 1ms 步长，摒弃 millis() 或 micros() 的复杂调用
    float Ts = 0.001f; 
    float alpha = filter->Tf / (filter->Tf + Ts);
    float y = alpha * filter->y_prev + (1.0f - alpha) * x;
    filter->y_prev = y;
    return y;
}

// ================= 算法实现：PID 控制器 (1ms 定步长) =================
__RAM_FUNC float JerryFOC_PID_Calc(PIDController* pid, float error) {
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
__RAM_FUNC static void setPwm(float da, float db, float dc) {
    // 直接接收 0~1 的占空比，彻底干掉电压换算的乘法套娃开销！
    float dc_a = _constrain(da, 0.0f , 1.0f );
    float dc_b = _constrain(db, 0.0f , 1.0f );
    float dc_c = _constrain(dc, 0.0f , 1.0f );

#if SWAP_MOTOR_PHASES
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dc_a * 8499));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dc_c * 8499));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc_b * 8499));
#else
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dc_a * 8499));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dc_b * 8499));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc_c * 8499));
#endif
}

__RAM_FUNC void JerryFOC_setPhaseVoltage(float Uq, float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el);

    // d/q 电压先做矢量限幅，避免 Ud、Uq 各自合法但合成矢量已经
    // 超出逆变器线性调制区。保留 5% 裕量给死区和母线波动。
    float voltage_limit = active_motor->bus_voltage * 0.548482756f; // 0.95/sqrt(3)
    float voltage_sq = Uq * Uq + Ud * Ud;
    float voltage_limit_sq = voltage_limit * voltage_limit;
    if (voltage_sq > voltage_limit_sq) {
        float scale = voltage_limit / sqrtf(voltage_sq);
        Uq *= scale;
        Ud *= scale;
    }

    float sin_angle = arm_sin_f32(angle_el);
    float cos_angle = arm_cos_f32(angle_el);
    
    // 1. 逆 Park 变换：将 d-q 轴电压转换为 alpha-beta 轴电压
    // 使用 CMSIS-DSP 查表极速算法替代标准的 sinf/cosf
    float Ualpha = -Uq * sin_angle + Ud * cos_angle;
    float Ubeta  =  Uq * cos_angle + Ud * sin_angle;

    // ----- 原 SPWM 调制方法 (已注释) -----
    // float Ua = Ualpha + voltage_power_supply/2;
    // float Ub = (sqrt(3)*Ubeta - Ualpha)/2 + voltage_power_supply/2;
    // float Uc = (-Ualpha - sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
    
    // ----- 原 SVPWM 调制方法 (Min-Max 零序注入法) 已注释 -----
    /*
    // 1. 逆 Clarke 变换：得到以中性点为参考的纯三相电压
    float Ua = Ualpha;
    float Ub = -0.5f * Ualpha + (sqrtf(3.0f)/2.0f) * Ubeta;
    float Uc = -0.5f * Ualpha - (sqrtf(3.0f)/2.0f) * Ubeta;
    
    // 2. 找出三相电压中的最大值和最小值
    float Umax = Ua > Ub ? (Ua > Uc ? Ua : Uc) : (Ub > Uc ? Ub : Uc);
    float Umin = Ua < Ub ? (Ua < Uc ? Ua : Uc) : (Ub < Uc ? Ub : Uc);
    
    // 3. 计算共模电压（零序分量）：使波形对称居中于 50% 占空比
    float Vbus = active_motor->bus_voltage;
    float Ucom = (Vbus / 2.0f) - (Umax + Umin) / 2.0f;
    
    // 4. 将零序电压注入，生成马鞍波
    Ua += Ucom;
    Ub += Ucom;
    Uc += Ucom;
    */

    // ----- 连续 SVPWM（Min-Max 零序注入）-----
    // 1. 逆 Clarke 变换：得到以中性点为参考的纯三相电压
    uint32_t start_cycles = DWT->CYCCNT;
    float Ua = Ualpha;
    float Ub = -0.5f * Ualpha + 0.86602540378f * Ubeta; // 0.86602540378f = sqrt(3)/2
    float Uc = -0.5f * Ualpha - 0.86602540378f * Ubeta;

    // 将相电压转为 0~1 的占空比
    // 将相电压转为 0~1 的占空比 (同样把除法变成乘法)
    float inv_Vbus = 1.0f / active_motor->bus_voltage;
    float da = Ua * inv_Vbus + 0.5f;
    float db = Ub * inv_Vbus + 0.5f;
    float dc = Uc * inv_Vbus + 0.5f;
    
    //DPWM-A调制会带来较大的共模电压干扰 先不要使用
    // 2. 使用三目运算符替代复杂的数学强制转换，利用编译器 IT 指令实现极速的无分支执行
    // // 计算各相到正边界(1)或中点(0.5)的最短距离
    // float dist_up_a = (da > 0.5f) ? (1.0f - da) : (0.5f - da);
    // float dist_up_b = (db > 0.5f) ? (1.0f - db) : (0.5f - db);
    // float dist_up_c = (dc > 0.5f) ? (1.0f - dc) : (0.5f - dc);

    // // 计算各相到负边界(0)或中点(0.5)的最短距离 (数学极速优化：恒定互补特性，直接做减法，砍掉所有的分支判断！)
    // float dist_down_a = 0.5f - dist_up_a;
    // float dist_down_b = 0.5f - dist_up_b;
    // float dist_down_c = 0.5f - dist_up_c;

    // // 3. 找出三相中的最小距离 (直接使用三目宏避免库函数调用的额外开销)
    // #define JERRY_MIN(x, y) ((x) < (y) ? (x) : (y))
    // float minV = JERRY_MIN(JERRY_MIN(dist_up_a, dist_up_b), dist_up_c);
    // float minV_0 = JERRY_MIN(JERRY_MIN(dist_down_a, dist_down_b), dist_down_c);
    // #undef JERRY_MIN

    // // 4. 判断并生成零序分量
    // float d_com = (minV <= minV_0) ? minV : -minV_0;

    // // 5. 将零序分量注入占空比，并直接传递给底层驱动（彻底砍掉电压反算的乘法开销）

    // 连续地将三相占空比范围居中到 0~1。原 DPWM 在钳位扇区切换时会产生
    // 较强的共模跳变，污染相电流采样并直接进入 SMO。
    float duty_max = da > db ? (da > dc ? da : dc) : (db > dc ? db : dc);
    float duty_min = da < db ? (da < dc ? da : dc) : (db < dc ? db : dc);
    float d_com = 0.5f - 0.5f * (duty_max + duty_min);

    // 将连续零序分量注入占空比。
    da += d_com;
    db += d_com;
    dc += d_com;

    // 使用最终限幅后的占空比反算实际可实现的 alpha/beta 电压。
    // SMO 必须看到逆变器真正施加的电压，而不是限幅前的 PI 指令。
    da = _constrain(da, 0.0f, 1.0f);
    db = _constrain(db, 0.0f, 1.0f);
    dc = _constrain(dc, 0.0f, 1.0f);
    float duty_common = (da + db + dc) * 0.3333333333f;
    float Ua_actual = (da - duty_common) * active_motor->bus_voltage;
    float Ub_actual = (db - duty_common) * active_motor->bus_voltage;
    float Uc_actual = (dc - duty_common) * active_motor->bus_voltage;
    g_shared_prev_Ualpha = Ua_actual;
    g_shared_prev_Ubeta = (Ub_actual - Uc_actual) * 0.5773502692f;
    g_shared_prev_Ud = g_shared_prev_Ualpha * cos_angle +
                       g_shared_prev_Ubeta * sin_angle;
    g_shared_prev_Uq = g_shared_prev_Ubeta * cos_angle -
                       g_shared_prev_Ualpha * sin_angle;

    setPwm(da, db, dc);
    dpwm_time_ns = (float)(DWT->CYCCNT - start_cycles) * (1000.0f / 170.0f);
}

// ================= 传感器标定与状态获取 =================
void JerryFOC_alignSensor(void) {
    // 编码器阻塞式标定期间暂停 ADC FOC 回调，否则 10kHz 电流环会
    // 立即覆盖下面的固定 d 轴标定电压。
    foc_fast_loop_enabled = 0U;
    // 使用编码器角度源执行零点对齐
    // 正确的对齐方式：在 D 轴（Ud）施加电压，迫使转子物理 D 轴对齐到电角度 0
    JerryFOC_setPhaseVoltage(0.0f, 3.0f, 0.0f);
    HAL_Delay(3000);

    // 只有拿到新鲜编码器数据才允许记录零点。若SPI接线/磁铁异常，
    // 保持未标定状态，后续快速环会持续撤掉PWM，而不会带着错误零点启动。
    if (AngleSrc_Encoder_IsSource(angle_source) &&
        MT6701_IsContinuous() && MT6701_IsHealthy()) {
        AngleSrc_Encoder_Align(0.0f);
    }

    JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
    foc_fast_loop_enabled = 1U;
}

float JerryFOC_getAngle(void) {
    if (angle_source) {
        return angle_source->getMechanicalAngle();
    }
    return 0.0f;
}

static uint8_t sensored_c2804_active(void) {
    return (active_motor == &MOTOR_C2804 &&
            AngleSrc_Encoder_IsSource(angle_source)) ? 1U : 0U;
}

static uint8_t sensored_control_ready(void) {
    if (!sensored_c2804_active()) {
        return 1U;
    }
    return (MT6701_IsContinuous() && MT6701_IsHealthy() &&
            AngleSrc_Encoder_IsAligned()) ? 1U : 0U;
}

static void reset_pid_state(PIDController *pid) {
    pid->integral_prev = 0.0f;
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
}

static void seed_sensored_velocity_pi(void) {
    sensored_velocity_reference = filtered_velocity_global;
    float error = sensored_velocity_reference - filtered_velocity_global;
    vel_loop_M0.integral_prev = _constrain(
        JerryFOC_Target_Iq - vel_loop_M0.P * error,
        -vel_loop_M0.limit, vel_loop_M0.limit);
    vel_loop_M0.error_prev = error;
    vel_loop_M0.output_prev = JerryFOC_Target_Iq;
}

static float update_velocity_reference(float command) {
    if (!sensored_c2804_active()) {
        sensored_velocity_reference = command;
        return command;
    }

    // 正反转不能一步跨过零速。方向相反时先以减速度收敛到0，
    // 下一控制周期再按加速度进入目标方向。
    float desired = command;
    float rate = SENSORED_ACCEL_LIMIT;
    if (sensored_velocity_reference * command < 0.0f &&
        fabsf(sensored_velocity_reference) > 1.0e-4f) {
        desired = 0.0f;
        rate = SENSORED_DECEL_LIMIT;
    } else if (fabsf(command) < fabsf(sensored_velocity_reference)) {
        rate = SENSORED_DECEL_LIMIT;
    }

    const float max_step = rate * 0.001f;
    float delta = desired - sensored_velocity_reference;
    if (delta > max_step) {
        delta = max_step;
    } else if (delta < -max_step) {
        delta = -max_step;
    }
    sensored_velocity_reference += delta;

    if (fabsf(desired - sensored_velocity_reference) < 1.0e-4f) {
        sensored_velocity_reference = desired;
    }
    return sensored_velocity_reference;
}


// ================= 闭环：模式、速度、扭矩设定与获取 =================
void JerryFOC_setMode(JerryFOC_ControlMode mode) {
    if (mode == current_control_mode) {
        return;
    }

    // C2804有感模式切换时用当前速度和当前输出初始化速度环，防止
    // Torque/Velocity/Position之间切换时旧积分项产生电压阶跃。
    if (sensored_c2804_active()) {
        if (mode == JERRYFOC_MODE_VELOCITY ||
            mode == JERRYFOC_MODE_POSITION) {
            seed_sensored_velocity_pi();
        } else {
            sensored_velocity_reference = filtered_velocity_global;
            reset_pid_state(&vel_loop_M0);
        }
        reset_pid_state(&pos_loop_M0);
    }
    current_control_mode = mode;
}
void JerryFOC_setPosition(float target_pos, float limit) { 
    // 获取当前的绝对机械角度
    float current_abs_pos = JerryFOC_getAngle();
    
    // 算出当前所处的“整圈起点基准坐标” (即向下取整的 2PI 倍数)
    // 例如当前 7.4pi (3.7圈)，除以 2pi = 3.7，floor(3.7) = 3，基准就是 6pi
    float base_angle = floorf(current_abs_pos / (2.0f * PI)) * 2.0f * PI;
    
    // 将用户传入的“相对本圈”的坐标加上基准坐标，得出绝对目标位置
    JerryFOC_Target_Position = base_angle + target_pos; 
    
    if (limit <= 0.0f) {
        limit = 30.0f; // 默认最大限幅 30 rad/s
    }
    velocity_limit_global = limit; 
    pos_loop_M0.limit = limit; 
}
void JerryFOC_setVelocity(float target_vel) { JerryFOC_Target_Velocity = target_vel; }
void JerryFOC_setCurrent(float target_Iq) { JerryFOC_Target_Iq = target_Iq; }
float JerryFOC_getVelocity(void) { return filtered_velocity_global; }
float JerryFOC_getPhaseCurrent_A(void) { return phase_current_a; }
float JerryFOC_getPhaseCurrent_B(void) { return phase_current_b; }
float JerryFOC_getIq(void) { return iq_current; }

// ================= Clarke 与 Park 变换 =================
__RAM_FUNC void JerryFOC_Clarke(float Ia, float Ib, float* Ialpha, float* Ibeta) {
    *Ialpha = Ia;
    // 提前计算常数，省去缓慢的 sqrtf 和除法
    *Ibeta = 0.577350269f * Ia + 1.154700538f * Ib;
}

__RAM_FUNC void JerryFOC_Park(float Ialpha, float Ibeta, float angle_el, float *Id, float *Iq) {
    float ct = arm_cos_f32(angle_el);
    float st = arm_sin_f32(angle_el);
    *Id = Ialpha * ct + Ibeta * st;
    *Iq = Ibeta * ct - Ialpha * st;
}

// 前馈必须使用“控制坐标系”的电角速度。OPENLOOP/SWITCH 阶段控制角主要
// 由 I/F 轨迹产生；完全闭环后才使用 SMO 的估算电角速度。
static float control_electrical_speed(const SMO_Observer *smo) {
    StartupState startup_state = Startup_GetState();
    if (startup_state == STARTUP_OPENLOOP ||
        startup_state == STARTUP_SWITCH) {
        return Startup_GetOpenLoopSpeed() *
               (float)active_motor->pole_pairs;
    }
    if (startup_state == STARTUP_CLOSED && smo) {
        return smo->omega_est_filtered;
    }
    return 0.0f;
}

// C2208 只有一个相电感参数，按表贴式 PMSM（Ld=Lq=Ls）进行 dq 解耦：
//   Ud_ff = -omega_e * Ls * Iq
//   Uq_ff =  omega_e * (psi_f + Ls * Id)
static void calculate_dq_feedforward(float omega_elec, float id, float iq,
                                     float *ud_ff, float *uq_ff) {
    *ud_ff = -omega_elec * active_motor->Ls * iq;
    *uq_ff =  omega_elec *
             (active_motor->psi_f + active_motor->Ls * id);
}

static void seed_bumpless_pi(float electrical_angle, float velocity_reference,
                             float iq_seed_limit) {
    float ct = arm_cos_f32(electrical_angle);
    float st = arm_sin_f32(electrical_angle);
    float applied_ud = g_shared_prev_Ualpha * ct + g_shared_prev_Ubeta * st;
    float applied_uq = g_shared_prev_Ubeta * ct - g_shared_prev_Ualpha * st;
    float ud_ff = 0.0f;
    float uq_ff = 0.0f;
    calculate_dq_feedforward(
        control_electrical_speed(AngleSrc_SMO_GetObserver()),
        id_current, iq_current, &ud_ff, &uq_ff);

    // 切换瞬间由当前实际 Iq 初始化。OPENLOOP->SWITCH 时允许调用者
    // 限制种子电流；电流 PI 的积分项会补偿目标变化，第一拍电压连续，
    // 随后电流平滑下降，避免把 0.4A 强拖电流当成闭环维持转矩。
    JerryFOC_Target_Iq = iq_current;
    if (iq_seed_limit > 0.0f) {
        JerryFOC_Target_Iq = _constrain(
            JerryFOC_Target_Iq, -iq_seed_limit, iq_seed_limit);
    }
    float err_q = JerryFOC_Target_Iq - iq_current;
    float err_d = -id_current;
    iq_loop_M0.integral_prev = _constrain(
        applied_uq - uq_ff - iq_loop_M0.P * err_q,
                                          -iq_loop_M0.limit, iq_loop_M0.limit);
    id_loop_M0.integral_prev = _constrain(
        applied_ud - ud_ff - id_loop_M0.P * err_d,
                                          -id_loop_M0.limit, id_loop_M0.limit);
    iq_loop_M0.error_prev = err_q;
    id_loop_M0.error_prev = err_d;
    iq_loop_M0.output_prev = applied_uq;
    id_loop_M0.output_prev = applied_ud;

    float vel_error = velocity_reference - filtered_velocity_global;
    vel_loop_M0.integral_prev = _constrain(
        JerryFOC_Target_Iq - vel_loop_M0.P * vel_error,
        -vel_loop_M0.limit, vel_loop_M0.limit);
    vel_loop_M0.error_prev = vel_error;
    vel_loop_M0.output_prev = JerryFOC_Target_Iq;
}

// 根据 PMSM 稳态电压方程计算当前转速下仍可实现的最大正向 Iq：
//   Vd = -omega_e * Ls * Iq
//   Vq =  Rs * Iq + omega_e * psi_f
// 并要求 sqrt(Vd^2 + Vq^2) 不超过 SVPWM 线性区。这样速度阶跃不会把
// 高阻电机的电流环长期推入电压饱和。
static float available_positive_iq(float omega_elec) {
    float omega = fabsf(omega_elec);
    float voltage_limit = active_motor->bus_voltage * 0.548482756f;
    float omega_L = omega * active_motor->Ls;
    float bemf = omega * active_motor->psi_f;
    float a = active_motor->Rs * active_motor->Rs + omega_L * omega_L;
    float b = 2.0f * active_motor->Rs * bemf;
    float c = bemf * bemf - voltage_limit * voltage_limit;
    float discriminant = b * b - 4.0f * a * c;

    if (a <= 0.0f || discriminant <= 0.0f) {
        return 0.0f;
    }

    float iq_limit = (-b + sqrtf(discriminant)) / (2.0f * a);
    return _constrain(iq_limit, 0.0f, active_motor->rated_current);
}

static float limit_velocity_iq(float iq_command, const SMO_Observer *smo,
                               float velocity_reference) {
    float omega_elec = smo ? smo->omega_est_filtered : 0.0f;
    float motoring_limit = available_positive_iq(omega_elec);
    float lower_limit = 0.0f;
    float upper_limit = 0.0f;

    // 纯无感运行时，超速后的反向 Iq 会让低惯量转子在几个采样周期内
    // 急剧减速，而 10ms 速度滤波仍报告原来的高速。随后速度 PI 会继续
    // 反向制动，PLL 相位误差翻转并丢锁。只允许与速度指令同向的电磁转矩；
    // 超速时输出零转矩，让转子自然滑行。
    if (velocity_reference > 0.0f) {
        upper_limit = motoring_limit;
    } else if (velocity_reference < 0.0f) {
        lower_limit = -motoring_limit;
    }

    float limited = _constrain(iq_command, lower_limit, upper_limit);

    // 外环动态限幅的回算抗饱和。否则速度目标不可达时积分器会一直顶在2 A，
    // 一旦角度或速度有扰动便产生很大的交变转矩。
    vel_loop_M0.integral_prev = _constrain(
        vel_loop_M0.integral_prev + 0.2f * (limited - iq_command),
        -vel_loop_M0.limit, vel_loop_M0.limit);
    vel_loop_M0.output_prev = limited;
    return limited;
}

// 由 ADC1 注入组完成中断严格按 10kHz 调用。
__RAM_FUNC void JerryFOC_run(void) {
    uint32_t start_cycles = DWT->CYCCNT;
    const float Ts_curr = 0.0001f;

    // 上电后在零输出状态采集两路 INA240 的真实零点。固定写死 1.65V
    // 会把参考源和运放偏差作为直流电流送入 SMO。
    if (adc_offset_samples < CURRENT_OFFSET_SAMPLES) {
        adc_offset_accum_a += ADC1->JDR1;
        adc_offset_accum_b += ADC2->JDR1;
        adc_offset_samples++;
        if (adc_offset_samples == CURRENT_OFFSET_SAMPLES) {
            adc_offset_count_a = (float)adc_offset_accum_a / (float)CURRENT_OFFSET_SAMPLES;
            adc_offset_count_b = (float)adc_offset_accum_b / (float)CURRENT_OFFSET_SAMPLES;
            JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
        }
        return;
    }

    // C2804有感模式必须同时满足“编码器在线且已经完成零点标定”。
    // 该保护位于10kHz快速环，编码器数据超时后不等待1kHz外环即可撤压。
    if (!sensored_control_ready()) {
        JerryFOC_Target_Iq = 0.0f;
        JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
        foc_loop_cycles = DWT->CYCCNT - start_cycles;
        foc_loop_time_ns = (float)foc_loop_cycles * (1000.0f / 170.0f);
        return;
    }

    float current_leg1 = ((float)ADC1->JDR1 - adc_offset_count_a) *
                         ADC_TO_VOLTS * VOLTS_TO_AMPS;
    float current_leg2 = ((float)ADC2->JDR1 - adc_offset_count_b) *
                         ADC_TO_VOLTS * VOLTS_TO_AMPS;

#if SWAP_MOTOR_PHASES
    phase_current_a = current_leg1;
    phase_current_b = -current_leg1 - current_leg2;
#else
    phase_current_a = current_leg1;
    phase_current_b = current_leg2;
#endif

    float Ialpha, Ibeta;
    JerryFOC_Clarke(phase_current_a, phase_current_b, &Ialpha, &Ibeta);

    // 观测器先使用上一 PWM 周期经占空比反算的实际电压更新。
    SMO_Observer *smo = AngleSrc_SMO_GetObserver();
    if (smo) {
        SMO_Update(smo, Ialpha, Ibeta,
                   g_shared_prev_Ualpha, g_shared_prev_Ubeta);
    }

    // 纯开环仅保留给显式 OpenLoop 调试命令，不参与无感启动。
    if (g_openloop_debug) {
        float speed_mech = JerryFOC_Target_Velocity;
        float uq_voltage = JerryFOC_Target_Iq;
        g_openloop_angle = _normalizeAngle(
            g_openloop_angle + speed_mech * (float)active_motor->pole_pairs * Ts_curr);

        float raw_Id, raw_Iq;
        JerryFOC_Park(Ialpha, Ibeta, g_openloop_angle, &raw_Id, &raw_Iq);
        float alpha = M0_Curr_Flt.Tf / (M0_Curr_Flt.Tf + Ts_curr);
        iq_current = alpha * M0_Curr_Flt.y_prev + (1.0f - alpha) * raw_Iq;
        M0_Curr_Flt.y_prev = iq_current;
        static float debug_id_prev = 0.0f;
        id_current = alpha * debug_id_prev + (1.0f - alpha) * raw_Id;
        debug_id_prev = id_current;

        JerryFOC_setPhaseVoltage(uq_voltage, 0.0f, g_openloop_angle);
        foc_loop_cycles = DWT->CYCCNT - start_cycles;
        foc_loop_time_ns = (float)foc_loop_cycles * (1000.0f / 170.0f);
        return;
    }

    // 启动状态机与 SMO 同频，消除原 1kHz 阶梯角度。
    Startup_Tick();

    float electrical_angle = 0.0f;
    if (angle_source) {
        electrical_angle = angle_source->getElectricalAngle();
    }

    float raw_Id, raw_Iq;
    JerryFOC_Park(Ialpha, Ibeta, electrical_angle, &raw_Id, &raw_Iq);
    float alpha = M0_Curr_Flt.Tf / (M0_Curr_Flt.Tf + Ts_curr);
    iq_current = alpha * M0_Curr_Flt.y_prev + (1.0f - alpha) * raw_Iq;
    M0_Curr_Flt.y_prev = iq_current;
    static float id_prev = 0.0f;
    id_current = alpha * id_prev + (1.0f - alpha) * raw_Id;
    id_prev = id_current;

    if (Startup_ConsumeSwitchTransition()) {
        // SWITCH 不再提前闭合不可靠的 SMO 速度环。以当前实际电流无扰
        // 初始化各 PI，随后由状态机的 0.4A->0.15A smoothstep 轨迹接管。
        seed_bumpless_pi(electrical_angle, Startup_GetOpenLoopSpeed(),
                         0.0f);
    }
    if (Startup_ConsumeClosedTransition()) {
        // 完全闭环后再无扰切到用户速度指令。
        seed_bumpless_pi(electrical_angle, JerryFOC_Target_Velocity, 0.0f);
    }

    StartupState startup_state = Startup_GetState();
    float target_iq = JerryFOC_Target_Iq;
    float target_id = 0.0f;
    if (startup_state == STARTUP_ALIGN) {
        target_iq = 0.0f;
        target_id = Startup_GetIdSetpoint();
    } else if (startup_state == STARTUP_OPENLOOP) {
        target_iq = Startup_GetIqSetpoint();
        // 让 CH6 表示启动阶段真正施加的 Iq，而不是遗留的速度环输出。
        JerryFOC_Target_Iq = target_iq;
    } else if (startup_state == STARTUP_SWITCH) {
        // 角度尚未完全交给 SMO 时，速度估计不能作为转矩闭环反馈。
        // 直接跟踪状态机生成的连续电流轨迹，完全 CLOSED 后才启用速度 PI。
        target_iq = Startup_GetIqSetpoint();
        JerryFOC_Target_Iq = target_iq;
    } else if (startup_state == STARTUP_FAULT) {
        target_iq = 0.0f;
        target_id = 0.0f;
        JerryFOC_Target_Iq = 0.0f;
    }

    float Uq = 0.0f;
    float Ud = 0.0f;
    float Uq_ff = 0.0f;
    float Ud_ff = 0.0f;
    uint8_t run_current_loop = flag_use_current_loop || Startup_IsActive();

    if (startup_state == STARTUP_FAULT) {
        Uq = 0.0f;
        Ud = 0.0f;
    } else if (!run_current_loop) {
        Uq = target_iq;
    } else {
        float err_q = target_iq - iq_current;
        float integ_q = iq_loop_M0.integral_prev +
            iq_loop_M0.I * Ts_curr * 0.5f * (err_q + iq_loop_M0.error_prev);
        integ_q = _constrain(integ_q, -iq_loop_M0.limit, iq_loop_M0.limit);
        iq_loop_M0.integral_prev = integ_q;
        iq_loop_M0.error_prev = err_q;
        float Uq_pi = _constrain(iq_loop_M0.P * err_q + integ_q,
                                 -iq_loop_M0.limit, iq_loop_M0.limit);

        float err_d = target_id - id_current;
        float integ_d = id_loop_M0.integral_prev +
            id_loop_M0.I * Ts_curr * 0.5f * (err_d + id_loop_M0.error_prev);
        integ_d = _constrain(integ_d, -id_loop_M0.limit, id_loop_M0.limit);
        id_loop_M0.integral_prev = integ_d;
        id_loop_M0.error_prev = err_d;
        float Ud_pi = _constrain(id_loop_M0.P * err_d + integ_d,
                                 -id_loop_M0.limit, id_loop_M0.limit);

        calculate_dq_feedforward(
            control_electrical_speed(smo),
            id_current, iq_current, &Ud_ff, &Uq_ff);
        Uq = Uq_pi + Uq_ff;
        Ud = Ud_pi + Ud_ff;
    }

    JerryFOC_setPhaseVoltage(Uq, Ud, electrical_angle);

    // 矢量限幅后的实际电压回算到 PI，抑制饱和时的积分累积。
    // Uq/Ud 是“PI+前馈”的总请求，因此回算误差会正确地反映前馈
    // 加入后剩余的真实电压裕量。
    if (run_current_loop && startup_state != STARTUP_FAULT) {
        iq_loop_M0.integral_prev = _constrain(
            iq_loop_M0.integral_prev + 0.2f * (g_shared_prev_Uq - Uq),
            -iq_loop_M0.limit, iq_loop_M0.limit);
        id_loop_M0.integral_prev = _constrain(
            id_loop_M0.integral_prev + 0.2f * (g_shared_prev_Ud - Ud),
            -id_loop_M0.limit, id_loop_M0.limit);
    }

    foc_loop_cycles = DWT->CYCCNT - start_cycles;
    foc_loop_time_ns = (float)foc_loop_cycles * (1000.0f / 170.0f);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        // SPI DMA不再在完成中断中自触发；由ADC节拍固定为10kHz，
        // 标定暂停FOC期间也继续刷新编码器样本。
        MT6701_Service();
        if (foc_fast_loop_enabled) {
            JerryFOC_run();
        }
    }
}

// ================= TIM2 1ms 定时器中断回调 =================
// 所有的速度计算、低通滤波、PID 运算均在此处以 1ms 的严格时序定频执行！
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        const uint8_t encoder_mode = sensored_c2804_active();
        if (encoder_mode) {
            // 多圈位置与速度只有这一处更新，getter只读取缓存，避免一次
            // 外环周期内多次读取角度导致跨圈状态被重复修改。
            AngleSrc_Encoder_Update(0.001f);
        }
        const uint8_t control_ready = sensored_control_ready();

        // 1. 速度计算：优先使用角度源直接输出的速度（SMO PLL 输出）
        float raw_velocity = 0.0f;
        if (angle_source) {
            if (angle_source->providesVelocity) {
                raw_velocity = angle_source->getVelocity();
            } else {
                // 编码器模式：差分计算机械角速度
                static float last_mechanical_angle = 0;
                static AngleSource *last_velocity_source = NULL;
                float current_mechanical_angle = angle_source->getMechanicalAngle();
                if (last_velocity_source == angle_source) {
                    raw_velocity = (current_mechanical_angle - last_mechanical_angle) / 0.001f;
                }
                last_mechanical_angle = current_mechanical_angle;
                last_velocity_source = angle_source;
            }
        }

        // 2. 速度低通滤波
        filtered_velocity_global = JerryFOC_LPF_Calc(&M0_Vel_Flt, raw_velocity);

        // 3. 获取当前机械角度（用于位置环）
        float current_mechanical_angle = 0.0f;
        if (angle_source) {
            current_mechanical_angle = angle_source->getMechanicalAngle();
        }

        // 4. 根据模式计算并串联 PID。编码器异常时清空外环状态，
        // 防止恢复通信后把超时期间积累的误差一次性施加到电机。
        if (encoder_mode && !control_ready) {
            JerryFOC_Target_Iq = 0.0f;
            sensored_velocity_reference = filtered_velocity_global;
            reset_pid_state(&vel_loop_M0);
            reset_pid_state(&pos_loop_M0);
        }
        else if (current_control_mode == JERRYFOC_MODE_POSITION) {
            // [外环] 位置环：输入位置误差，输出目标速度
            float error_pos = JerryFOC_Target_Position - current_mechanical_angle;
            JerryFOC_Target_Velocity = JerryFOC_PID_Calc(&pos_loop_M0, error_pos);

            // [内环] 速度环：输入速度误差，输出目标电流(实际为 Uq)
            float velocity_reference =
                update_velocity_reference(JerryFOC_Target_Velocity);
            float error_vel = velocity_reference - filtered_velocity_global;

            if (Startup_GetState() == STARTUP_IDLE || Startup_GetState() == STARTUP_CLOSED) {
                JerryFOC_Target_Iq = JerryFOC_PID_Calc(&vel_loop_M0, error_vel);
            }
        }
        else if (current_control_mode == JERRYFOC_MODE_VELOCITY) {
            // 仅使用速度环：输入速度误差，输出目标电流(实际为 Uq)
            StartupState startup_state = Startup_GetState();
            float velocity_reference =
                update_velocity_reference(JerryFOC_Target_Velocity);
            float error_vel = velocity_reference - filtered_velocity_global;

            // OPENLOOP/SWITCH 的 Iq 均由10kHz启动轨迹产生，速度 PI 只在
            // 有感 IDLE 或无感 CLOSED 状态运行。
            if (startup_state == STARTUP_IDLE ||
                startup_state == STARTUP_CLOSED) {
                float iq_command = JerryFOC_PID_Calc(&vel_loop_M0, error_vel);
                if (startup_state == STARTUP_CLOSED) {
                    SMO_Observer *smo = AngleSrc_SMO_GetObserver();
                    if (smo && SMO_IsSignalValid(smo)) {
                        float estimated_speed_mech =
                            smo->omega_est_filtered /
                            (float)active_motor->pole_pairs;
                        float speed_abs = fabsf(estimated_speed_mech);
                        float reference_abs = fabsf(JerryFOC_Target_Velocity);
                        float overspeed_margin = 0.75f * reference_abs;
                        if (overspeed_margin < 20.0f) {
                            overspeed_margin = 20.0f;
                        }

                        if (speed_abs <= reference_abs + overspeed_margin) {
                            JerryFOC_Target_Iq = limit_velocity_iq(
                                iq_command, smo, JerryFOC_Target_Velocity);
                        } else {
                            // 无编码器时无法区分真实超速与 PLL 伪速度；两种情况下
                            // 安全动作都应是撤掉转矩，禁止速度 PI 反向制动造成抽动。
                            JerryFOC_Target_Iq = 0.0f;
                            vel_loop_M0.integral_prev = 0.0f;
                            vel_loop_M0.output_prev = 0.0f;
                        }
                    } else {
                        // 角度不可观测时撤掉转矩，禁止错误电角度下继续强驱而高频震动。
                        JerryFOC_Target_Iq = 0.0f;
                        vel_loop_M0.output_prev = 0.0f;
                    }
                } else {
                    // 有感模式停留在 IDLE，保留其原有的电压/电流控制语义。
                    JerryFOC_Target_Iq = iq_command;
                }
            }
        }

        // 5. 每 10ms (100Hz) 通过串口发送一次 JustFloat 协议数据包给 VOFA+
        static int print_counter = 0;
        if (++print_counter >= 10) {
            print_counter = 0;

            // JustFloat 数据结构 (扩展为16个通道)
            static struct Frame {
                float fdata[16];
                unsigned char tail[4];
            } frame;

            // 获取 SMO 诊断数据
            float smo_omega = 0.0f;
            SMO_Observer *smo_diag = AngleSrc_SMO_GetObserver();
            if (smo_diag) {
                smo_omega = smo_diag->omega_est_filtered;
            }

            // CH0~CH9
            frame.fdata[0] = JerryFOC_Target_Velocity;     // CH0: 目标速度
            frame.fdata[1] = filtered_velocity_global;     // CH1: 实际速度
            frame.fdata[2] = iq_current;                   // CH2: 实际 Iq
            frame.fdata[3] = id_current;                   // CH3: 实际 Id
            frame.fdata[4] = phase_current_a;              // CH4: A相电流
            frame.fdata[5] = phase_current_b;              // CH5: B相电流
            frame.fdata[6] = JerryFOC_Target_Iq;           // CH6: 速度环目标 Iq
            frame.fdata[7] = smo_omega;                    // CH7: SMO 估算电角速度
            frame.fdata[8] = (float)Startup_GetState();    // CH8: 启动状态 (0~5)
            StartupFaultReason fault_reason = Startup_GetFaultReason();
            frame.fdata[9] = fault_reason != STARTUP_FAULT_NONE
                ? -(float)fault_reason
                : (smo_diag ? (float)SMO_IsSignalValid(smo_diag) : 0.0f);
            // CH9: 正常时 0/1=BEMF无效/有效；故障时 -1=超时, -2=反转, -3=观测器丢失
            frame.fdata[10] = smo_diag ? sqrtf(smo_diag->bemf_sq) : 0.0f; // CH10: BEMF幅值
            frame.fdata[11] = Startup_GetState() == STARTUP_CLOSED
                ? (smo_diag ? smo_diag->pll_error : 0.0f)
                : Startup_GetPhaseError();
            // CH11: OPENLOOP为SMO/开环相差，SWITCH为SMO/实际换相角误差，
            // CLOSED为实时PLL归一化误差
            frame.fdata[12] = smo_diag ? smo_diag->Ealpha_est : 0.0f;
            frame.fdata[13] = smo_diag ? smo_diag->Ebeta_est : 0.0f;
            frame.fdata[14] = smo_diag ? smo_diag->bemf_ratio : 0.0f;
            frame.fdata[15] = smo_diag ? smo_diag->pll_error : 0.0f;
            // CH12/13: SMO反电动势α/β；CH14: |E|/(ψf·|ωe|)；
            // CH15: 原始归一化PLL误差（所有启动状态均可观察）

            if (encoder_mode) {
                // C2804有感模式下SMO诊断量没有意义，改为编码器与给定
                // 规划器状态，方便直接判断SPI失联、标定遗漏和速度突变。
                frame.fdata[7] = sensored_velocity_reference;              // CH7: 斜坡后速度给定
                frame.fdata[8] = (float)MT6701_IsHealthy();                // CH8: 编码器健康状态
                frame.fdata[9] = (float)MT6701_GetSampleAgeCycles();       // CH9: 样本年龄(100us/计数)
                frame.fdata[10] = (float)MT6701_GetRawAngle();             // CH10: 14位原始角度
                frame.fdata[11] = (float)MT6701_GetErrorCounter();         // CH11: SPI/DMA累计错误
                frame.fdata[12] = angle_source->getElectricalAngle();      // CH12: 电角度(rad)
                frame.fdata[13] = current_mechanical_angle;                // CH13: 多圈机械角(rad)
                frame.fdata[14] = (float)MT6701_GetStatus();               // CH14: MT6701 Mg[3:0]
                frame.fdata[15] = (float)AngleSrc_Encoder_IsAligned();      // CH15: 零点标定完成
            }
            
            // 填充固定帧尾
            frame.tail[0] = 0x00;
            frame.tail[1] = 0x00;
            frame.tail[2] = 0x80;
            frame.tail[3] = 0x7f;
            
            HAL_UART_Transmit_IT(&huart1, (uint8_t*)&frame, sizeof(frame));
        }
    }
}
