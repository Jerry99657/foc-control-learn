#ifndef __JERRYFOC_H
#define __JERRYFOC_H

#include "main.h"
#include "tim.h"
#include <math.h>
#include "MT6701.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ================= 低通滤波器结构体 =================
typedef struct {
    float Tf;             // 滤波时间常数
    float y_prev;         // 上一次的输出值
} LowPassFilter;

// ================= PID 控制器结构体 =================
typedef struct {
    float P;              // 比例系数
    float I;              // 积分系数
    float D;              // 微分系数
    float output_ramp;    // 输出变化率限幅
    float limit;          // 输出绝对值限幅
    
    float error_prev;     // 上一次的误差
    float output_prev;    // 上一次的输出
    float integral_prev;  // 上一次的积分项
} PIDController;

// ================= 全局控制器对象 =================
extern LowPassFilter M0_Vel_Flt;
extern PIDController vel_loop_M0;

// ================= 接口函数声明 =================
// 基础 FOC 底层配置
void JerryFOC_alignSensor(void);
void JerryFOC_setPhaseVoltage(float Uq, float Ud, float angle_el);

// PID 与滤波计算函数 (将被 TIM2 1ms 中断调用)
float JerryFOC_LPF_Calc(LowPassFilter* filter, float x);
float JerryFOC_PID_Calc(PIDController* pid, float error);

// 速度控制高层接口
void JerryFOC_setVelocity(float target_vel);

// 供外部获取状态
float JerryFOC_getVelocity(void);
float JerryFOC_getAngle(void);

// 主循环高速调用接口
void JerryFOC_run(void);

#endif
