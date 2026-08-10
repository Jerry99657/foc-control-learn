#ifndef __ANGLE_SRC_ENCODER_H
#define __ANGLE_SRC_ENCODER_H

#include "angle_source.h"

// 创建编码器角度源实例
// 返回指向静态 AngleSource 的指针，全局唯一
AngleSource* AngleSrc_Encoder_Init(int pole_pairs);

// 编码器零点标定（替代原 JerryFOC_alignSensor 的效果）
void AngleSrc_Encoder_Align(float electrical_angle_rad);

// 由1kHz运动控制任务唯一调用，更新多圈机械角和原始机械速度。
// 其余getter均为纯读取，不再修改跨圈状态。
void AngleSrc_Encoder_Update(float sample_period_s);
uint8_t AngleSrc_Encoder_IsAligned(void);
uint8_t AngleSrc_Encoder_IsSource(const AngleSource *source);

#endif /* __ANGLE_SRC_ENCODER_H */
