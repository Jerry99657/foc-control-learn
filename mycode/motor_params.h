#ifndef __MOTOR_PARAMS_H
#define __MOTOR_PARAMS_H

#include <stdint.h>

// ================= 电机参数结构体 =================
// 每款电机一个实例，运行时通过 JerryFOC_selectMotor() 切换
typedef struct {
    const char *name;           // 电机标识名
    int pole_pairs;             // 极对数

    // === 电气参数 ===
    float Rs;                   // 相电阻 (Ω)
    float Ls;                   // 相电感 (H)
    float psi_f;                // 永磁体磁链 (Wb)，有编码器时可填 0
    float bus_voltage;          // 母线电压 (V)

    // === 额定值 ===
    float rated_current;        // 额定电流 (A)
    float max_current;          // 最大电流 (A)
    float rated_speed_mech;     // 额定机械角速度 (rad/s)
    float max_speed_mech;       // 最高机械角速度 (rad/s)

    // === 电流环 PI（d/q 轴共用） ===
    float cur_bandwidth;        // 目标带宽 (rad/s)，文档参考值
    float cur_P;                // 比例增益 (V/A)
    float cur_I;                // 积分增益 (V/(A·s))
    float cur_limit;            // 输出电压限幅 (V)
    float cur_ramp;             // 输出变化率限幅 (V/s)，0=不限幅

    // === 速度环 PI ===
    float vel_P;
    float vel_I;
    float vel_limit;            // 输出限幅 (V)，通常 = bus_voltage / √3
    float vel_ramp;             // 输出变化率限幅 (V/s)
    float vel_filter_Tf;        // 速度 LPF 时间常数 (s)

    // === 位置环 P ===
    float pos_P;
    float pos_limit;            // 输出限幅 (rad/s)
    float pos_ramp;

    // === SMO 参数（仅无感模式使用） ===
    float smo_Kslide;           // 滑模增益
    float smo_wc;               // 反电动势 LPF 截止频率 (rad/s)
    float smo_boundary;         // 饱和函数边界层 (A)
    float smo_min_bemf;         // PLL 最小有效反电动势幅值 (V)
    float pll_Kp;               // PLL 比例增益
    float pll_Ki;               // PLL 积分增益
    float pll_limit;            // PLL 输出限幅 (rad/s 电角速度)
} MotorParams;

// ================= 预置电机实例（定义在 motor_params.c） =================
extern const MotorParams MOTOR_C2804;  // C2804: Rs=2.3Ω, Ls=0.86mH, ωc=250
extern const MotorParams MOTOR_C2208;  // C2208: Rs=10.6Ω, Ls=5.3mH, ωc=1000

#endif /* __MOTOR_PARAMS_H */
