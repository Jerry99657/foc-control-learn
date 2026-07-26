#include "angle_src_encoder.h"
#include "MT6701.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ========== 多圈角度跟踪（从 JerryFOC.c 迁移） ==========
static float prev_Angle = 0;
static float full_rotations = 0;
static float mechanical_zero_offset = 0;
static float zero_electric_angle = 0;
static int   pp = 7;             // 极对数
static int   DIR = 1;

static float _normalizeAngle(float angle) {
    float a = fmodf(angle, 2.0f * PI);
    return a >= 0 ? a : (a + 2.0f * PI);
}

// ========== 接口实现 ==========

static float angle_meas_electrical(void) {
    return _normalizeAngle((float)(DIR * pp) * Angle - zero_electric_angle);
}

static float angle_meas_mechanical(void) {
    float current_Angle = Angle;
    float d_angle = current_Angle - prev_Angle;

    // 跨越 2π 的跳变检测
    if (d_angle > PI) {
        full_rotations -= 1.0f;
    } else if (d_angle < -PI) {
        full_rotations += 1.0f;
    }
    prev_Angle = current_Angle;

    return full_rotations * 2.0f * PI + current_Angle - mechanical_zero_offset;
}

static float angle_meas_velocity(void) {
    // 速度由外部 TIM2 中断中的差分+滤波计算
    // 这里返回 0，实际速度由 JerryFOC 的 filtered_velocity_global 提供
    return 0.0f;
}

// ========== 静态实例 ==========
static AngleSource encoder_src = {
    .getElectricalAngle = angle_meas_electrical,
    .getMechanicalAngle = angle_meas_mechanical,
    .getVelocity        = angle_meas_velocity,
    .providesVelocity   = 0U,
};

AngleSource* AngleSrc_Encoder_Init(int pole_pairs) {
    pp = pole_pairs;
    prev_Angle = Angle;
    full_rotations = 0;
    mechanical_zero_offset = Angle;
    zero_electric_angle = 0;
    return &encoder_src;
}

void AngleSrc_Encoder_Align(float electrical_angle_rad) {
    // 记录当前传感器电角度作为零点偏移
    zero_electric_angle = _normalizeAngle((float)(DIR * pp) * Angle - electrical_angle_rad);

    // 重置多圈跟踪
    prev_Angle = Angle;
    full_rotations = 0;
    mechanical_zero_offset = Angle;
}
