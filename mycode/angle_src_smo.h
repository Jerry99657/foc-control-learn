#ifndef __ANGLE_SRC_SMO_H
#define __ANGLE_SRC_SMO_H

#include "angle_source.h"
#include "smo.h"
#include "motor_params.h"

// 创建 SMO 角度源（无感模式专用）
// motor: 当前电机参数
// sample_freq_hz: SMO 更新频率，应与 FOC 电流环频率一致（5000Hz）
AngleSource* AngleSrc_SMO_Create(const MotorParams *motor, float sample_freq_hz);

// 获取内部 SMO 观测器指针（供外部读取诊断数据）
SMO_Observer* AngleSrc_SMO_GetObserver(void);

// 手动设置初始角度（启动对齐后调用）
void AngleSrc_SMO_SetAngle(float theta_init);

// 重置 SMO 状态
void AngleSrc_SMO_Reset(void);

#endif /* __ANGLE_SRC_SMO_H */
