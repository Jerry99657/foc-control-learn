#ifndef __STARTUP_H
#define __STARTUP_H

#include <stdint.h>

// ================= 无感启动状态机 =================
typedef enum {
    STARTUP_IDLE = 0,       // 空闲，未启动
    STARTUP_ALIGN,          // 固定 d 轴电流完成转子定位
    STARTUP_OPENLOOP,       // I/F 开环强拖并等待 SMO 稳定
    STARTUP_SWITCH,         // 开环角度到 SMO 角度平滑混合
    STARTUP_CLOSED,         // 完全使用 SMO 角度
    STARTUP_FAULT,          // 超时或观测器无法稳定
} StartupState;

typedef enum {
    STARTUP_FAULT_NONE = 0,
    STARTUP_FAULT_TIMEOUT = 1,
    STARTUP_FAULT_REVERSE = 2,
    STARTUP_FAULT_OBSERVER_LOST = 3,
} StartupFaultReason;

typedef struct {
    float align_current;        // d 轴对齐电流 (A)
    float align_duration;       // 对齐持续时间 (s)
    float align_angle;          // 对齐电角度 (rad)

    float openloop_Iq;          // I/F 强拖 q 轴电流 (A)
    float openloop_ramp;        // 机械角速度斜率 (rad/s^2)
    float switch_speed;         // 机械切换速度 (rad/s)
    float switch_duration;      // 角度混合时间 (s)

    float lock_duration;        // SMO 条件连续满足时间 (s)
    float switch_timeout;       // 等待 SMO 锁定的最大时间 (s)
    float lock_loss_timeout;    // 闭环后观测器连续失效的容忍时间 (s)
    float lock_speed_tolerance; // 估算/开环机械速度允许差值 (rad/s)
    float lock_phase_tolerance; // SMO/开环电角度允许差值 (rad)
} StartupConfig;

// sample_freq_hz 必须等于 SMO/FOC 的实际更新频率。
void Startup_Init(int pole_pairs, const StartupConfig *cfg, float sample_freq_hz);
StartupState Startup_GetState(void);
StartupFaultReason Startup_GetFaultReason(void);

// 每个 FOC/ADC 周期调用一次。
void Startup_Tick(void);

float Startup_GetOutputAngle(void);
void Startup_Begin(void);
void Startup_Stop(void);

// 请求安全切换；仍需满足 SMO 有效性条件，不再绕过锁定判据。
void Startup_ForceClosed(void);

float Startup_GetOpenLoopAngle(void);
float Startup_GetOpenLoopSpeed(void);
float Startup_GetPhaseError(void);
// OPENLOOP 返回固定强拖电流；SWITCH 返回与角度混合权重同步的平滑电流。
float Startup_GetIqSetpoint(void);
float Startup_GetIdSetpoint(void);
uint8_t Startup_IsActive(void);

// 闭环切换事件由 10kHz 电流环消费，用于无扰初始化 PI。
uint8_t Startup_ConsumeSwitchTransition(void);
uint8_t Startup_ConsumeClosedTransition(void);

#endif /* __STARTUP_H */
