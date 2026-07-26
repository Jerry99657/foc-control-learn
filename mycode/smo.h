#ifndef __SMO_H
#define __SMO_H

#include "motor_params.h"

// ================= 滑模观测器 + PLL 锁相环 =================
typedef struct {
    // === 电机参数（从 MotorParams 复制，避免指针依赖） ===
    float Rs;               // 相电阻 (Ω)
    float Ls;               // 相电感 (H)
    float psi_f;            // 永磁体磁链 (Wb)
    float Ts;               // 采样周期 (s) = 1/Fs

    // === SMO 增益 ===
    float Kslide;           // 滑模增益
    float Kslf;             // 低通滤波系数 α（Tustin 法）
    float one_minus_alpha_over_2; // (1-α)/2，Tustin 系数

    // === 饱和函数边界层 ===
    float boundary;         // 边界层厚度 (A)
    float min_bemf_sq;      // PLL 最小有效反电动势幅值平方 (V^2)

    // === 状态变量 ===
    float Ialpha_est;       // 估算 α 轴电流
    float Ibeta_est;        // 估算 β 轴电流
    float Ealpha_est;       // 滤波后 α 轴反电动势
    float Ebeta_est;        // 滤波后 β 轴反电动势

    // LPF 历史值（Tustin 需要）
    float Zalpha_prev;      // Zα(k-1)
    float Zbeta_prev;       // Zβ(k-1)

    // === PLL 锁相环 ===
    float theta_est;        // 估算电角度 (rad，有延迟)
    float theta_est_comp;   // 补偿滤波延迟后的真实电角度 (rad)
    float omega_est;        // 估算电角速度 (rad/s)
    float omega_est_filtered; // 滤波后的电角速度 (rad/s，用于相位补偿)
    float integ_pll;        // PLL PI 积分项
    float Kp_pll;           // PLL 比例增益
    float Ki_pll;           // PLL 积分增益
    float pll_limit;        // PLL 输出限幅 (rad/s)
    float wc;               // LPF 截止频率 (rad/s)
    float pll_error;        // 归一化 PLL 相位误差
    float bemf_sq;          // 滤波后反电动势幅值平方 (V^2)
    float expected_bemf;    // 由 ψf·|ωe| 得到的理论反电动势幅值 (V)
    float bemf_ratio;       // 实测/理论反电动势幅值比
    uint16_t signal_counter;// 连续有效样本计数
    uint16_t signal_bad_counter; // 连续低反电动势样本计数
    uint8_t bemf_consistent;// BEMF 幅值与当前速度是否满足物理一致性
    uint8_t signal_valid;   // 反电动势是否连续达到有效阈值

    // === 电机模型系数（预计算） ===
    float A;                // (1 - Ts*Rs/Ls)
    float B;                // Ts/Ls
} SMO_Observer;

// ========== API ==========

// 初始化 SMO，传入电机参数和采样频率
void SMO_Init(SMO_Observer *smo, const MotorParams *motor, float sample_freq_hz);

// 单步更新（每次 ADC 采样后调用，10kHz）
// 输入：实测 Iα,Iβ，指令电压 Vα,Vβ
// 输出：更新 smo->theta_est, smo->omega_est
void SMO_Update(SMO_Observer *smo, float Ialpha, float Ibeta, float Valpha, float Vbeta);

// 重置状态（电机切换或启动时调用）
void SMO_Reset(SMO_Observer *smo);

// 手动设置初始角度（启动对齐后调用）
void SMO_SetAngle(SMO_Observer *smo, float theta_init);

// 设置 PLL 初始角度和电角速度（仅用于启动前馈种子）
void SMO_SetAngleSpeed(SMO_Observer *smo, float theta_init, float omega_elec_init);

// 强制反电动势判定回到无效，但保留电流/反电动势滤波状态。
void SMO_InvalidateSignal(SMO_Observer *smo);

// 将 PLL 的原始速度、滤波速度和积分状态限制在指定窗口内。
// 仅供开环转闭环的混合阶段使用，防止弱反电动势下 PLL 跳到伪速度。
void SMO_ClampSpeed(SMO_Observer *smo, float center_speed, float half_width);

// 判断反电动势是否已连续达到有效阈值
uint8_t SMO_IsSignalValid(const SMO_Observer *smo);

#endif /* __SMO_H */
