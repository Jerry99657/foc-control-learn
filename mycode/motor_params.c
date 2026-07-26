#include "motor_params.h"

// ===== 电机 0：C2804（低阻小电感，云台/航模类） =====
// 参数来源于 JerryFOC.c 现有 PID 反推：Ls=0.86mH, Rs=2.3Ω, ωc=250
const MotorParams MOTOR_C2804 = {
    .name = "C2804",
    .pole_pairs = 7,
    .Rs = 2.3f,         .Ls = 0.00086f,        .psi_f = 0.0f,
    .bus_voltage = 12.0f,
    .rated_current = 2.0f,      .max_current = 5.0f,
    .rated_speed_mech = 300.0f, .max_speed_mech = 600.0f,
    .cur_bandwidth = 250.0f,
    .cur_P = 0.215f,    .cur_I = 575.0f,
    .cur_limit = 12.0f, .cur_ramp = 100000.0f,
    .vel_P = 0.05f,     .vel_I = 0.5f,
    .vel_limit = 6.93f, .vel_ramp = 0.0f,
    .vel_filter_Tf = 0.005f,
    .pos_P = 2.0f,      .pos_limit = 30.0f,    .pos_ramp = 0.0f,
    .smo_Kslide = 10.0f,    .smo_wc = 5000.0f,     .smo_boundary = 0.05f,
    .smo_min_bemf = 0.10f,
    .pll_Kp = 200.0f,       .pll_Ki = 10000.0f,    .pll_limit = 6000.0f,
};

// ===== 电机 1：C2208（高阻大电感，14W 小功率，东兴威） =====
// 线电阻 21.2Ω → 相电阻 10.6Ω，相电感 5.3mH（用户确认）
// Ke=0.0796 V/(rad/s) 线反电动势 → ψf=Ke/(√3·p)=0.00657 Wb
const MotorParams MOTOR_C2208 = {
    .name = "C2208",
    .pole_pairs = 7,
    .Rs = 10.6f,        .Ls = 0.0053f,          .psi_f = 0.00657f,
    .bus_voltage = 12.0f,
    .rated_current = 0.8f,      .max_current = 4.5f,
    .rated_speed_mech = 188.5f, .max_speed_mech = 301.6f,
    .cur_bandwidth = 400.0f,
    .cur_P = 2.0f,      .cur_I = 4000.0f,
    .cur_limit = 6.93f, .cur_ramp = 100000.0f,
    // SMO 速度含有开关纹波，速度环必须明显慢于 PLL；过高带宽会把估算纹波
    // 重新变成转矩纹波，造成实机震动。
    .vel_P = 0.02f,     .vel_I = 0.08f,
    .vel_limit = 2.0f,  .vel_ramp = 0.0f,
    // SMO 内部速度已经经过 10ms LPF；外环再用 20ms 会增加约一个
    // 抽动周期内的相位滞后，改为 10ms 兼顾纹波与动态响应。
    .vel_filter_Tf = 0.010f,
    .pos_P = 2.0f,      .pos_limit = 30.0f,    .pos_ramp = 0.0f,
    // 原 Kslide=10、wc=2000 会把电流环瞬态作为强反电动势送入 PLL。
    // 5V 滑模增益仍覆盖 Speed:40 时约 1.84V 的真实相反电动势，
    // 同时降低边界层斜率和观测器高频纹波。
    .smo_Kslide = 5.0f,     .smo_wc = 1000.0f,     .smo_boundary = 0.20f,
    // 实机停转噪声底约 0.035V；0.08V 保留超过 2 倍余量，
    // 同时避免 15rad/s 强拖时有效标志在阈值边缘反复跳变。
    .smo_min_bemf = 0.08f,
    // 串口数据中 PLL 误差与速度的主振荡均约 6.67Hz，和原
    // wn=40rad/s (6.37Hz) 几乎重合。降到 wn=20rad/s、zeta=1，
    // 避免 PLL 摆动通过速度 PI 重新变成转矩脉冲。
    .pll_Kp = 40.0f,        .pll_Ki = 400.0f,      .pll_limit = 3000.0f,
};
