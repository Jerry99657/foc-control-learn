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
// 速度外环全局对象
extern LowPassFilter M0_Vel_Flt;
extern PIDController vel_loop_M0;

// 位置外环全局对象
extern PIDController pos_loop_M0;

// 电流环全局对象
extern LowPassFilter M0_Curr_Flt;
extern PIDController id_loop_M0;
extern PIDController iq_loop_M0;

// ================= 控制模式枚举 =================
typedef enum {
    JERRYFOC_MODE_TORQUE = 0,    // 纯扭矩/电流模式 (只闭环 Iq)
    JERRYFOC_MODE_VELOCITY = 1,  // 速度模式 (速度外环 + Iq 内环)
    JERRYFOC_MODE_POSITION = 2   // 位置模式 (位置外环 + 速度内环 + Iq 最内环)
} JerryFOC_ControlMode;

// ================= 接口函数声明 =================
// 启动音效
void JerryFOC_playStartupSound(void);

// 模式与目标设置
void JerryFOC_setMode(JerryFOC_ControlMode mode);
void JerryFOC_useCurrentLoop(uint8_t enable); // 0: 电压控制(默认), 1: 启用 PI 电流环
void JerryFOC_setVelocity(float target_vel);
void JerryFOC_setCurrent(float target_Iq);
void JerryFOC_alignSensor(void);
void JerryFOC_setPhaseVoltage(float Uq, float Ud, float angle_el);

// PID 与滤波计算函数
float JerryFOC_LPF_Calc(LowPassFilter* filter, float x);
float JerryFOC_PID_Calc(PIDController* pid, float error);

// Clarke 与 Park 变换
void JerryFOC_Clarke(float Ia, float Ib, float* Ialpha, float* Ibeta);
void JerryFOC_Park(float Ialpha, float Ibeta, float angle_el, float *Id, float *Iq);


void JerryFOC_setPosition(float target_pos, float limit);

// 供外部获取状态
float JerryFOC_getVelocity(void);
float JerryFOC_getAngle(void);
float JerryFOC_getPhaseCurrent_A(void);
float JerryFOC_getPhaseCurrent_B(void);
float JerryFOC_getIq(void);

// 主循环高速调用接口
void JerryFOC_run(void);

#endif
