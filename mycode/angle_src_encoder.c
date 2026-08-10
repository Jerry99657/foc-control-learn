#include "angle_src_encoder.h"
#include "MT6701.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ========== 编码器缓存：仅 AngleSrc_Encoder_Update() 修改 ==========
static float previous_sensor_angle = 0.0f;
static float mechanical_angle = 0.0f;
static float mechanical_velocity = 0.0f;
static float zero_electric_angle = 0.0f;
static uint32_t last_sample_counter = 0U;
static uint8_t encoder_aligned = 0U;
static uint8_t encoder_initialized = 0U;
static uint8_t encoder_was_healthy = 0U;
static int pp = 7;             // 极对数
static int DIR = 1;

// 4ms位置差分窗口把MT6701单计数对应的速度量化阶梯降至1/4，
// 随后仍由JerryFOC中的5ms一阶低通抑制剩余噪声。
#define ENCODER_VELOCITY_WINDOW 4U
static float velocity_position_history[ENCODER_VELOCITY_WINDOW] = {0.0f};
static uint8_t velocity_history_index = 0U;
static uint8_t velocity_history_count = 0U;

static void reset_velocity_estimator(void) {
    for (uint8_t i = 0U; i < ENCODER_VELOCITY_WINDOW; i++) {
        velocity_position_history[i] = mechanical_angle;
    }
    velocity_history_index = 0U;
    velocity_history_count = 0U;
    mechanical_velocity = 0.0f;
}

static float _normalizeAngle(float angle) {
    float a = fmodf(angle, 2.0f * PI);
    return a >= 0 ? a : (a + 2.0f * PI);
}

// ========== 接口实现 ==========

static float angle_meas_electrical(void) {
    // Angle 由SPI DMA回调以单个32位对齐写入；本地快照避免同一次
    // 计算中跨越两个样本。getter本身不再修改任何跟踪状态。
    float sensor_angle = Angle;
    return _normalizeAngle((float)(DIR * pp) * sensor_angle - zero_electric_angle);
}

static float angle_meas_mechanical(void) {
    return mechanical_angle;
}

static float angle_meas_velocity(void) {
    return mechanical_velocity;
}

// ========== 静态实例 ==========
static AngleSource encoder_src = {
    .getElectricalAngle = angle_meas_electrical,
    .getMechanicalAngle = angle_meas_mechanical,
    .getVelocity        = angle_meas_velocity,
    .providesVelocity   = 1U,
};

AngleSource* AngleSrc_Encoder_Init(int pole_pairs) {
    pp = pole_pairs > 0 ? pole_pairs : 1;
    previous_sensor_angle = Angle;
    mechanical_angle = 0.0f;
    mechanical_velocity = 0.0f;
    zero_electric_angle = 0.0f;
    last_sample_counter = MT6701_GetSampleCounter();
    encoder_aligned = 0U;
    encoder_initialized = 1U;
    encoder_was_healthy = 0U;
    reset_velocity_estimator();
    return &encoder_src;
}

void AngleSrc_Encoder_Align(float electrical_angle_rad) {
    float sensor_angle = Angle;
    // 记录当前传感器电角度作为零点偏移
    zero_electric_angle = _normalizeAngle(
        (float)(DIR * pp) * sensor_angle - electrical_angle_rad);

    // 标定完成位置定义为机械零点，并从同一份样本重新开始速度估算。
    previous_sensor_angle = sensor_angle;
    mechanical_angle = 0.0f;
    mechanical_velocity = 0.0f;
    last_sample_counter = MT6701_GetSampleCounter();
    encoder_initialized = 1U;
    encoder_aligned = 1U;
    encoder_was_healthy = 1U;
    reset_velocity_estimator();
}

void AngleSrc_Encoder_Update(float sample_period_s) {
    if (!encoder_initialized || sample_period_s <= 0.0f ||
        !MT6701_IsHealthy()) {
        encoder_was_healthy = 0U;
        reset_velocity_estimator();
        return;
    }

    uint32_t sample_counter = MT6701_GetSampleCounter();
    if (sample_counter == last_sample_counter) {
        return;
    }
    last_sample_counter = sample_counter;

    float sensor_angle = Angle;
    if (!encoder_was_healthy) {
        // 失联期间经过的角度和时间未知，恢复首帧只用于重新同步，
        // 不能把整段位移除以1ms形成虚假速度尖峰。
        previous_sensor_angle = sensor_angle;
        encoder_was_healthy = 1U;
        reset_velocity_estimator();
        return;
    }

    float delta = sensor_angle - previous_sensor_angle;
    if (delta > PI) {
        delta -= 2.0f * PI;
    } else if (delta < -PI) {
        delta += 2.0f * PI;
    }
    previous_sensor_angle = sensor_angle;

    delta *= (float)DIR;
    mechanical_angle += delta;

    float oldest_position =
        velocity_position_history[velocity_history_index];
    velocity_position_history[velocity_history_index] = mechanical_angle;
    velocity_history_index = (uint8_t)(
        (velocity_history_index + 1U) % ENCODER_VELOCITY_WINDOW);

    if (velocity_history_count < ENCODER_VELOCITY_WINDOW) {
        velocity_history_count++;
    }
    mechanical_velocity = (mechanical_angle - oldest_position) /
        (sample_period_s * (float)velocity_history_count);
}

uint8_t AngleSrc_Encoder_IsAligned(void) {
    return encoder_aligned;
}

uint8_t AngleSrc_Encoder_IsSource(const AngleSource *source) {
    return source == &encoder_src ? 1U : 0U;
}
