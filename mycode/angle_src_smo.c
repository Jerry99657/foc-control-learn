#include "angle_src_smo.h"
#include "startup.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ========== 静态实例 ==========
static SMO_Observer smo_observer;
static int smo_pole_pairs = 7;

// ========== 接口实现 ==========

static float smo_electrical_angle(void) {
    // 启动期间：使用状态机混合角度（对齐/开环/切换）
    StartupState st = Startup_GetState();
    if (st != STARTUP_IDLE) {
        return Startup_GetOutputAngle();
    }
    // 闭环后：直接使用 SMO PLL 的相位补偿角度（10kHz 实时）
    return smo_observer.theta_est_comp;
}

static float smo_mechanical_angle(void) {
    // 无感模式下无绝对机械角度，用电角度换算（仅用于速度估算）
    return smo_observer.theta_est / (float)smo_pole_pairs;
}

static float smo_velocity(void) {
    StartupState st = Startup_GetState();
    if (st == STARTUP_ALIGN || st == STARTUP_FAULT) {
        return 0.0f;
    }
    if (st == STARTUP_OPENLOOP) {
        // 纯开环阶段没有独立可信速度，使用 I/F 轨迹作为反馈。
        return Startup_GetOpenLoopSpeed();
    }
    if (st == STARTUP_SWITCH) {
        if (SMO_IsSignalValid(&smo_observer)) {
            // 进入混合阶段后必须让速度环看到受 Startup_Tick 限幅后的
            // SMO 速度。原先继续返回固定开环速度会使速度误差恒为零，
            // 速度 PI 的积分项一直保持约 0.4A，转子超过同步速度后仍
            // 持续加速，最终相位越过 90 度并产生正反抽动。
            return smo_observer.omega_est_filtered /
                   (float)smo_pole_pairs;
        }
        // BEMF 短时无效时保持开环轨迹，避免速度反馈突然跌到零。
        return Startup_GetOpenLoopSpeed();
    }
    if (!SMO_IsSignalValid(&smo_observer)) {
        return 0.0f;
    }
    return smo_observer.omega_est_filtered / (float)smo_pole_pairs;
}

static AngleSource smo_src = {
    .getElectricalAngle = smo_electrical_angle,
    .getMechanicalAngle = smo_mechanical_angle,
    .getVelocity        = smo_velocity,
    .providesVelocity   = 1U,
};

// ========== API 实现 ==========

AngleSource* AngleSrc_SMO_Create(const MotorParams *motor, float sample_freq_hz) {
    smo_pole_pairs = motor->pole_pairs;
    SMO_Init(&smo_observer, motor, sample_freq_hz);
    return &smo_src;
}

SMO_Observer* AngleSrc_SMO_GetObserver(void) {
    return &smo_observer;
}

void AngleSrc_SMO_SetAngle(float theta_init) {
    SMO_SetAngle(&smo_observer, theta_init);
}

void AngleSrc_SMO_Reset(void) {
    SMO_Reset(&smo_observer);
}
