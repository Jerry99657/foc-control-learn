#ifndef __ANGLE_SOURCE_H
#define __ANGLE_SOURCE_H

#include <stdint.h>

// ================= 角度源抽象接口 =================
// FOC 引擎不关心角度来自编码器还是 SMO 观测器
typedef struct AngleSource {
    float (*getElectricalAngle)(void);   // 获取电角度 (rad, [0, 2π])
    float (*getMechanicalAngle)(void);   // 获取机械角度 (rad, 连续多圈)
    float (*getVelocity)(void);          // 获取机械角速度 (rad/s)
    uint8_t providesVelocity;            // 1=速度可直接使用，0=需对机械角差分
} AngleSource;

#endif /* __ANGLE_SOURCE_H */
