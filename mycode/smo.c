#include "smo.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "arm_math.h"

// 定义放入 CCMRAM 极速执行的宏
#ifndef __RAM_FUNC
#define __RAM_FUNC __attribute__((section(".RamFunc")))
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ========== 饱和函数（减少抖振） ==========
__RAM_FUNC static inline float sat(float x, float delta) {
    if (x > delta) return 1.0f;
    if (x < -delta) return -1.0f;
    return x / delta;
}

// ========== 快速平方根倒数算法 ==========
// 使用 memcpy 做位转换，避免 Release 优化下违反 strict-aliasing。
__RAM_FUNC static inline float fast_inv_sqrt(float number) {
    uint32_t i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = number * 0.5f;
    y  = number;
    memcpy(&i, &y, sizeof(i));
    i  = 0x5f3759df - ( i >> 1 );
    memcpy(&y, &i, sizeof(y));
    y  = y * ( threehalfs - ( x2 * y * y ) );
    return y;
}

// ========== 归一化角度到 [0, 2π) ==========
__RAM_FUNC static inline float normalize_angle(float a) {
    // 采用极速的 浮点->整型(VCVT) 汇编指令求余 (乘法替代除法)
    float res = a - (int)(a * 0.159154943091895f) * (2.0f * PI);
    return res >= 0.0f ? res : (res + 2.0f * PI);
}

// ========== 初始化 ==========
void SMO_Init(SMO_Observer *smo, const MotorParams *motor, float sample_freq_hz) {
    memset(smo, 0, sizeof(SMO_Observer));

    smo->Rs    = motor->Rs;
    smo->Ls    = motor->Ls;
    smo->psi_f = motor->psi_f;
    smo->Ts    = 1.0f / sample_freq_hz;

    smo->Kslide  = motor->smo_Kslide;
    smo->boundary = motor->smo_boundary;
    smo->min_bemf_sq = motor->smo_min_bemf * motor->smo_min_bemf;

    // === Tustin 双线性变换 LPF 系数 ===
    // α = (2 - wc·Ts) / (2 + wc·Ts)
    float wcTs = motor->smo_wc * smo->Ts;
    float alpha = (2.0f - wcTs) / (2.0f + wcTs);
    smo->Kslf = alpha;
    smo->one_minus_alpha_over_2 = (1.0f - alpha) * 0.5f;
    smo->wc = motor->smo_wc;

    // === PLL 参数 ===
    smo->Kp_pll = motor->pll_Kp;
    smo->Ki_pll = motor->pll_Ki;
    smo->pll_limit = motor->pll_limit;

    // === 预计算电机模型系数 ===
    // A = 1 - Ts * Rs / Ls
    // B = Ts / Ls
    smo->A = 1.0f - smo->Ts * smo->Rs / smo->Ls;
    smo->B = smo->Ts / smo->Ls;

    smo->theta_est = 0.0f;
    smo->omega_est = 0.0f;
}

// ========== 单步更新 ==========
__RAM_FUNC void SMO_Update(SMO_Observer *smo, float Ialpha, float Ibeta, float Valpha, float Vbeta) {
    // === 1. 滑模电流观测器 ===
    // 电流误差
    float Ialpha_err = smo->Ialpha_est - Ialpha;
    float Ibeta_err  = smo->Ibeta_est  - Ibeta;

    // 滑模控制量（饱和函数）
    float Zalpha = smo->Kslide * sat(Ialpha_err, smo->boundary);
    float Zbeta  = smo->Kslide * sat(Ibeta_err,  smo->boundary);

    // 观测器状态更新（前向欧拉，稳定可控）
    // Îα(k+1) = A·Îα(k) + B·(Vα(k) - Zα(k))
    smo->Ialpha_est = smo->A * smo->Ialpha_est + smo->B * (Valpha - Zalpha);
    smo->Ibeta_est  = smo->A * smo->Ibeta_est  + smo->B * (Vbeta  - Zbeta);

    // === 2. 反电动势提取（Tustin 一阶 LPF） ===
    // Ê(k) = α·Ê(k-1) + (1-α)/2 · (Z(k) + Z(k-1))
    smo->Ealpha_est = smo->Kslf * smo->Ealpha_est
                    + smo->one_minus_alpha_over_2 * (Zalpha + smo->Zalpha_prev);
    smo->Ebeta_est  = smo->Kslf * smo->Ebeta_est
                    + smo->one_minus_alpha_over_2 * (Zbeta  + smo->Zbeta_prev);

    smo->Zalpha_prev = Zalpha;
    smo->Zbeta_prev  = Zbeta;

    // === 3. PLL 锁相环角度/速度估计 ===
    // 误差信号：ε = -Êα·cos(θ̂) - Êβ·sin(θ̂)
    // 对于小误差：ε ≈ ψf·ωe·(θ - θ̂)
    float cos_theta = arm_cos_f32(smo->theta_est);
    float sin_theta = arm_sin_f32(smo->theta_est);
    float epsilon = -smo->Ealpha_est * cos_theta - smo->Ebeta_est * sin_theta;

    // 归一化：只在反电动势连续有效时运行 PLL。静止/低速阶段冻结
    // 积分器，避免电流零偏和开关噪声把 PLL 拉到错误速度。
    float E_sq = smo->Ealpha_est * smo->Ealpha_est + smo->Ebeta_est * smo->Ebeta_est;

    // 对反电动势包络再做 2ms 低通，避免滑模开关纹波的单个谷值
    // 把“连续 100 点有效”永久打断。
    const float bemf_alpha = smo->Ts / (0.002f + smo->Ts);
    smo->bemf_sq += bemf_alpha * (E_sq - smo->bemf_sq);

    // 仅凭“BEMF 幅值超过噪声底”无法排除电流环/PWM 瞬态造成的伪信号。
    // 对真实 PMSM 还必须近似满足 |E| = ψf·|ωe|。实机数据表明伪锁时
    // 二者比值可从 0.06 跳到 3.17，因此使用较宽的 0.35~2.5 窗口：
    // 它允许参数误差和动态过程，但能拒绝明显不可能的速度/BEMF组合。
    float omega_abs = fabsf(smo->omega_est_filtered);
    smo->expected_bemf = smo->psi_f * omega_abs;
    float expected_bemf_sq = smo->expected_bemf * smo->expected_bemf;
    float ratio_sq = 0.0f;
    if (expected_bemf_sq > 1.0e-8f) {
        ratio_sq = smo->bemf_sq / expected_bemf_sq;
        smo->bemf_ratio = sqrtf(ratio_sq);
    } else {
        smo->bemf_ratio = 0.0f;
    }
    smo->bemf_consistent =
        (expected_bemf_sq >= smo->min_bemf_sq &&
         ratio_sq >= 0.1225f &&   // 0.35^2
         ratio_sq <= 6.25f)       // 2.5^2
        ? 1U : 0U;

    // 带迟滞的有效性判定：连续 10ms 同时满足幅值和物理一致性才有效；
    // 已有效后连续 5ms 低幅值或不一致才清除锁定标志。
    float bemf_low_sq = 0.49f * smo->min_bemf_sq;
    if (!smo->signal_valid) {
        if (smo->bemf_sq >= smo->min_bemf_sq &&
            smo->bemf_consistent) {
            if (smo->signal_counter < 100U) smo->signal_counter++;
        } else {
            smo->signal_counter = 0U;
        }
        if (smo->signal_counter >= 100U) {
            smo->signal_valid = 1U;
            smo->signal_bad_counter = 0U;
        }
    } else {
        if (smo->bemf_sq < bemf_low_sq ||
            !smo->bemf_consistent) {
            if (smo->signal_bad_counter < 50U) smo->signal_bad_counter++;
        } else {
            smo->signal_bad_counter = 0U;
        }
        if (smo->signal_bad_counter >= 50U) {
            smo->signal_valid = 0U;
            smo->signal_counter = 0U;
        }
    }
    smo->pll_error = 0.0f;

    // signal_valid 是已经过 10ms 准入和 5ms 退出消抖的最终状态。
    // 有效期间即使 BEMF 比例短时越界，也继续以降低后的置信度跟踪 PLL；
    // 否则第一个比例坏样本就会冻结 PLL，实际绕过上面的退出消抖。
    if (smo->signal_valid) {
        // 用滤波后的包络归一化；迟滞保持有效期间，瞬时 E_sq 可能
        // 出现很深的开关纹波谷值，直接取其倒平方根会把噪声放大。
        float normalization_sq = smo->bemf_sq;
        if (normalization_sq < bemf_low_sq) {
            normalization_sq = bemf_low_sq;
        }
        epsilon *= fast_inv_sqrt(normalization_sq);
        // 理想归一化相位误差为 sin(Δθ)，绝对值不应超过 1。
        // 包络滤波与瞬时反电动势不同步时可能暂时超过该范围，必须钳位，
        // 否则较大的 Ki 会把开关噪声积分成数百 rad/s 的伪速度。
        if (epsilon >  1.0f) epsilon =  1.0f;
        if (epsilon < -1.0f) epsilon = -1.0f;
        smo->pll_error = epsilon;

        // 弱反电动势刚过有效阈值时，归一化会同时放大噪声。按反电动势
        // 置信度调度 PLL：幅值为阈值时 wn 约为标称值的一半，达到阈值
        // 两倍后才使用完整增益。Kp 按 sqrt(confidence)、Ki 按
        // confidence 缩放，可近似保持阻尼比不变。
        float confidence = smo->bemf_sq / (4.0f * smo->min_bemf_sq);
        if (confidence > 1.0f) confidence = 1.0f;
        if (confidence < 0.10f) confidence = 0.10f;

        // 幅值尖峰不应反过来提高 PLL 增益。比值偏离 1 越远，
        // 一致性置信度越低；在允许窗口边缘也会主动减小环路带宽。
        float consistency_confidence =
            ratio_sq <= 1.0f ? ratio_sq : (1.0f / ratio_sq);
        if (consistency_confidence < 0.10f) {
            consistency_confidence = 0.10f;
        }
        confidence *= consistency_confidence;
        float proportional_confidence = sqrtf(confidence);

        // PI 控制器
        smo->integ_pll += smo->Ki_pll * confidence * smo->Ts * epsilon;

        // 积分抗饱和
        if (smo->integ_pll >  smo->pll_limit) smo->integ_pll =  smo->pll_limit;
        if (smo->integ_pll < -smo->pll_limit) smo->integ_pll = -smo->pll_limit;

        smo->omega_est = smo->Kp_pll * proportional_confidence * epsilon
                       + smo->integ_pll;

        // 速度限幅
        if (smo->omega_est >  smo->pll_limit) smo->omega_est =  smo->pll_limit;
        if (smo->omega_est < -smo->pll_limit) smo->omega_est = -smo->pll_limit;
    } else {
        // 低反电动势时只保留启动种子，并缓慢衰减，禁止噪声积分。
        smo->omega_est *= 0.999f;
        smo->integ_pll = smo->omega_est;
    }

    // 角度积分 (积分本身有低通特性，直接用原始 omega_est)
    smo->theta_est += smo->omega_est * smo->Ts;
    smo->theta_est = normalize_angle(smo->theta_est);
    
    // 平滑电角速度 (用于相位补偿，防止滑模高频噪声注入角度)
    // alpha = Ts / (Tf + Ts), Tf = 0.010s，抑制滑模纹波造成的 PLL 速度尖峰。
    float alpha_vel = smo->Ts / (0.010f + smo->Ts);
    smo->omega_est_filtered = (1.0f - alpha_vel) * smo->omega_est_filtered + alpha_vel * smo->omega_est;

    // 相位补偿：抵消低通滤波器引入的相位延迟
    // 延迟角 = arctan(电角速度 / 截止频率)
    float phase_delay = atanf(smo->omega_est_filtered / smo->wc);
    smo->theta_est_comp = normalize_angle(smo->theta_est + phase_delay);
}

// ========== 重置 ==========
void SMO_Reset(SMO_Observer *smo) {
    smo->Ialpha_est = 0.0f;
    smo->Ibeta_est  = 0.0f;
    smo->Ealpha_est = 0.0f;
    smo->Ebeta_est  = 0.0f;
    smo->Zalpha_prev = 0.0f;
    smo->Zbeta_prev  = 0.0f;
    smo->theta_est = 0.0f;
    smo->omega_est = 0.0f;
    smo->omega_est_filtered = 0.0f;
    smo->integ_pll = 0.0f;
    smo->theta_est_comp = 0.0f;
    smo->pll_error = 0.0f;
    smo->bemf_sq = 0.0f;
    smo->expected_bemf = 0.0f;
    smo->bemf_ratio = 0.0f;
    smo->signal_counter = 0U;
    smo->signal_bad_counter = 0U;
    smo->bemf_consistent = 0U;
    smo->signal_valid = 0U;
}

// ========== 手动设角度 ==========
void SMO_SetAngle(SMO_Observer *smo, float theta_init) {
    SMO_SetAngleSpeed(smo, theta_init, 0.0f);
}

void SMO_SetAngleSpeed(SMO_Observer *smo, float theta_init, float omega_elec_init) {
    smo->theta_est = normalize_angle(theta_init);
    smo->theta_est_comp = smo->theta_est;
    smo->omega_est = omega_elec_init;
    smo->omega_est_filtered = omega_elec_init;
    smo->integ_pll = omega_elec_init;
    smo->pll_error = 0.0f;
}

void SMO_InvalidateSignal(SMO_Observer *smo) {
    smo->signal_counter = 0U;
    smo->signal_bad_counter = 0U;
    smo->bemf_consistent = 0U;
    smo->signal_valid = 0U;
    smo->pll_error = 0.0f;
}

void SMO_ClampSpeed(SMO_Observer *smo, float center_speed, float half_width) {
    if (!smo || half_width <= 0.0f) {
        return;
    }

    float lower = center_speed - half_width;
    float upper = center_speed + half_width;
    if (smo->omega_est < lower) smo->omega_est = lower;
    if (smo->omega_est > upper) smo->omega_est = upper;
    if (smo->omega_est_filtered < lower) smo->omega_est_filtered = lower;
    if (smo->omega_est_filtered > upper) smo->omega_est_filtered = upper;
    if (smo->integ_pll < lower) smo->integ_pll = lower;
    if (smo->integ_pll > upper) smo->integ_pll = upper;
}

uint8_t SMO_IsSignalValid(const SMO_Observer *smo) {
    // bemf_consistent 是未经消抖的瞬时诊断量，不能在这里再次与最终
    // signal_valid 相与，否则所有上层调用者都会绕过 5ms 退出消抖。
    return (smo && smo->signal_valid) ? 1U : 0U;
}
