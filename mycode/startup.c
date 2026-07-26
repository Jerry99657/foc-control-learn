#include "startup.h"
#include "angle_src_smo.h"
#include <math.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// SMO 的 signal_valid 本身已经包含 5ms 失效迟滞。SWITCH 再确认
// 20ms 后立即无跳变退回开环，避免在错误角度下零转矩等待 80ms。
#define SWITCH_INVALID_TIMEOUT 0.020f

static volatile StartupState state = STARTUP_IDLE;
static volatile StartupFaultReason fault_reason = STARTUP_FAULT_NONE;
static StartupConfig config;
static int pp = 7;
static float Ts = 0.0001f;

static float state_elapsed = 0.0f;
static float startup_elapsed = 0.0f;
static float lock_elapsed = 0.0f;
static float invalid_elapsed = 0.0f;
static float openloop_theta_elec = 0.0f;
static float openloop_speed_mech = 0.0f;
static float blend_weight = 0.0f;
static float output_angle = 0.0f;
static float phase_error = 0.0f;
static float switch_angle_offset = 0.0f;
static float reverse_elapsed = 0.0f;
static volatile uint8_t force_switch_requested = 0U;
static volatile uint8_t switch_transition_pending = 0U;
static volatile uint8_t closed_transition_pending = 0U;

static inline float normalize_angle(float a) {
    while (a >= 2.0f * PI) a -= 2.0f * PI;
    while (a < 0.0f)       a += 2.0f * PI;
    return a;
}

static inline float wrap_pm_pi(float a) {
    if (a > PI)       a -= 2.0f * PI;
    else if (a < -PI) a += 2.0f * PI;
    return a;
}

static void advance_openloop_angle(void) {
    openloop_theta_elec = normalize_angle(
        openloop_theta_elec + openloop_speed_mech * (float)pp * Ts);
}

static float observer_angle(const SMO_Observer *smo) {
    return normalize_angle(smo->theta_est_comp);
}

static uint8_t observer_direction_reversed(const SMO_Observer *smo) {
    // 当前串口 Speed 协议只支持正向无感启动。反电动势有效后，
    // 若独立估算速度持续反向，说明转子掉步或 PLL 极性错误。
    // 原来单点低于 -2rad/s 就触发故障，PLL 伪锁的负尖峰会误报 -2。
    if (SMO_IsSignalValid(smo) &&
        smo->omega_est_filtered < -(2.0f * (float)pp)) {
        reverse_elapsed += Ts;
    } else {
        reverse_elapsed = 0.0f;
    }
    return reverse_elapsed >= 0.02f ? 1U : 0U;
}

static uint8_t observer_matches_openloop(const SMO_Observer *smo) {
    if (!SMO_IsSignalValid(smo)) {
        phase_error = 0.0f;
        return 0U;
    }

    float estimated_speed_mech = smo->omega_est_filtered / (float)pp;
    float speed_error = fabsf(estimated_speed_mech - openloop_speed_mech);
    phase_error = wrap_pm_pi(observer_angle(smo) - openloop_theta_elec);

    return (speed_error <= config.lock_speed_tolerance &&
            fabsf(phase_error) <= config.lock_phase_tolerance &&
            fabsf(smo->pll_error) <= 0.9f) ? 1U : 0U;
}

static uint8_t observer_healthy_during_switch(const SMO_Observer *smo) {
    if (!SMO_IsSignalValid(smo)) {
        return 0U;
    }

    float estimated_speed_mech = smo->omega_est_filtered / (float)pp;
    float minimum_forward_speed = 0.5f * config.switch_speed;
    float maximum_capture_speed =
        config.switch_speed + 3.0f * config.lock_speed_tolerance;

    // phase_error 在 SWITCH 主流程中定义为“SMO 角度 - 实际换相角度”。
    // 混合开始后输出角速度逐渐交给 SMO，转子会从 15rad/s 正常加速；
    // 不能再与已经退出控制的固定开环速度比较。只检查正向捕获区、
    // 实际换相相位和与 CLOSED 相同的 PLL 安全边界。
    return (estimated_speed_mech >= minimum_forward_speed &&
            estimated_speed_mech <= maximum_capture_speed &&
            fabsf(phase_error) <= config.lock_phase_tolerance + 0.2f &&
            fabsf(smo->pll_error) <= 0.98f) ? 1U : 0U;
}

void Startup_Init(int pole_pairs, const StartupConfig *cfg, float sample_freq_hz) {
    pp = pole_pairs > 0 ? pole_pairs : 1;
    memset(&config, 0, sizeof(config));
    if (cfg) {
        memcpy(&config, cfg, sizeof(config));
    }

    if (sample_freq_hz <= 0.0f) sample_freq_hz = 10000.0f;
    Ts = 1.0f / sample_freq_hz;
    if (config.lock_duration <= 0.0f) config.lock_duration = 0.10f;
    if (config.switch_timeout <= 0.0f) config.switch_timeout = 3.0f;
    if (config.lock_loss_timeout <= 0.0f) config.lock_loss_timeout = 0.05f;
    if (config.lock_speed_tolerance <= 0.0f) config.lock_speed_tolerance = 3.0f;
    if (config.lock_phase_tolerance <= 0.0f) config.lock_phase_tolerance = 1.2f;
    if (config.switch_duration <= 0.0f) config.switch_duration = 0.5f;

    state = STARTUP_IDLE;
    fault_reason = STARTUP_FAULT_NONE;
    state_elapsed = 0.0f;
    startup_elapsed = 0.0f;
    lock_elapsed = 0.0f;
    invalid_elapsed = 0.0f;
    openloop_theta_elec = normalize_angle(config.align_angle);
    openloop_speed_mech = 0.0f;
    blend_weight = 0.0f;
    output_angle = openloop_theta_elec;
    phase_error = 0.0f;
    switch_angle_offset = 0.0f;
    reverse_elapsed = 0.0f;
    force_switch_requested = 0U;
    switch_transition_pending = 0U;
    closed_transition_pending = 0U;
}

StartupState Startup_GetState(void) {
    return state;
}

StartupFaultReason Startup_GetFaultReason(void) {
    return fault_reason;
}

void Startup_Begin(void) {
    AngleSrc_SMO_Reset();
    SMO_SetAngleSpeed(AngleSrc_SMO_GetObserver(), config.align_angle, 0.0f);

    state = STARTUP_ALIGN;
    fault_reason = STARTUP_FAULT_NONE;
    state_elapsed = 0.0f;
    startup_elapsed = 0.0f;
    lock_elapsed = 0.0f;
    invalid_elapsed = 0.0f;
    openloop_theta_elec = normalize_angle(config.align_angle);
    openloop_speed_mech = 0.0f;
    blend_weight = 0.0f;
    output_angle = openloop_theta_elec;
    phase_error = 0.0f;
    switch_angle_offset = 0.0f;
    reverse_elapsed = 0.0f;
    force_switch_requested = 0U;
    switch_transition_pending = 0U;
    closed_transition_pending = 0U;
}

void Startup_Stop(void) {
    state = STARTUP_IDLE;
    fault_reason = STARTUP_FAULT_NONE;
    force_switch_requested = 0U;
    switch_transition_pending = 0U;
    closed_transition_pending = 0U;
    openloop_speed_mech = 0.0f;
    switch_angle_offset = 0.0f;
    phase_error = 0.0f;
    reverse_elapsed = 0.0f;
}

void Startup_ForceClosed(void) {
    force_switch_requested = 1U;
}

float Startup_GetOpenLoopAngle(void) {
    return openloop_theta_elec;
}

float Startup_GetOpenLoopSpeed(void) {
    return openloop_speed_mech;
}

float Startup_GetPhaseError(void) {
    return phase_error;
}

float Startup_GetIqSetpoint(void) {
    StartupState st = state;
    if (st == STARTUP_OPENLOOP) {
        return config.openloop_Iq;
    }
    if (st == STARTUP_SWITCH) {
        // 角度混合期间保持转矩连续：从已经验证能稳定强拖的开环电流
        // 平滑下降到 37.5%（C2208 配置下 0.15A）。不能在 2->3 的
        // 同一拍把 0.4A 截成 0.08A，否则反电势会在混合完成前消失。
        const float hold_ratio = 0.375f;
        float current_ratio =
            1.0f - (1.0f - hold_ratio) * blend_weight;
        return config.openloop_Iq * current_ratio;
    }
    return 0.0f;
}

float Startup_GetIdSetpoint(void) {
    return state == STARTUP_ALIGN ? config.align_current : 0.0f;
}

uint8_t Startup_IsActive(void) {
    StartupState st = state;
    return (st == STARTUP_ALIGN || st == STARTUP_OPENLOOP ||
            st == STARTUP_SWITCH) ? 1U : 0U;
}

uint8_t Startup_ConsumeClosedTransition(void) {
    if (closed_transition_pending) {
        closed_transition_pending = 0U;
        return 1U;
    }
    return 0U;
}

uint8_t Startup_ConsumeSwitchTransition(void) {
    if (switch_transition_pending) {
        switch_transition_pending = 0U;
        return 1U;
    }
    return 0U;
}

void Startup_Tick(void) {
    SMO_Observer *smo = AngleSrc_SMO_GetObserver();

    switch (state) {
    case STARTUP_IDLE:
        return;

    case STARTUP_ALIGN:
        state_elapsed += Ts;
        output_angle = normalize_angle(config.align_angle);
        if (state_elapsed >= config.align_duration) {
            state = STARTUP_OPENLOOP;
            state_elapsed = 0.0f;
            startup_elapsed = 0.0f;
            openloop_speed_mech = 0.0f;
            openloop_theta_elec = output_angle;
            // ALIGN 的电流阶跃不是反电动势。不清空观测器会把对齐瞬态
            // 带入 OPENLOOP，造成刚起转就误判为反转。
            SMO_Reset(smo);
            SMO_SetAngleSpeed(smo, openloop_theta_elec, 0.0f);
        }
        return;

    case STARTUP_OPENLOOP: {
        state_elapsed += Ts;
        startup_elapsed += Ts;
        if (openloop_speed_mech < config.switch_speed) {
            openloop_speed_mech += config.openloop_ramp * Ts;
            if (openloop_speed_mech > config.switch_speed) {
                openloop_speed_mech = config.switch_speed;
            }
        }
        advance_openloop_angle();
        output_angle = openloop_theta_elec;

        // 低速时真实反电动势不可观，电流阶跃和死区误差会伪造有效信号。
        // 到达 60% 切换速度前只保留开环 PLL 种子，不允许累积有效计数。
        if (openloop_speed_mech < 0.6f * config.switch_speed) {
            SMO_InvalidateSignal(smo);
        }

        // 开环期间控制角本来就不依赖 SMO。只要 BEMF/速度物理一致性
        // 失败，就把 PLL 重新种到已知的 I/F 轨迹，禁止伪速度继续自由滑行。
        if (!SMO_IsSignalValid(smo)) {
            SMO_SetAngleSpeed(smo, openloop_theta_elec,
                              openloop_speed_mech * (float)pp);
        }

        // 掉步后继续旋转定子磁场会把转子猛拉向反方向，必须立即停止。
        if (openloop_speed_mech >= config.switch_speed &&
            observer_direction_reversed(smo)) {
            fault_reason = STARTUP_FAULT_REVERSE;
            state = STARTUP_FAULT;
            return;
        }

        uint8_t observer_locked = 0U;
        if (openloop_speed_mech >= config.switch_speed) {
            observer_locked = observer_matches_openloop(smo);
        }
        if (observer_locked) {
            lock_elapsed += Ts;
        } else {
            // 滑模反电动势必然含有 PWM 纹波；单个坏样本不应把整段
            // 锁定历史清零。坏样本以 2 倍速率扣减，仍需要超过 2/3
            // 的样本同时满足 BEMF/速度/相位/PLL 条件才能进入 SWITCH。
            lock_elapsed -= 2.0f * Ts;
            if (lock_elapsed < 0.0f) lock_elapsed = 0.0f;
        }

        if (openloop_speed_mech >= config.switch_speed &&
            (lock_elapsed >= config.lock_duration ||
             (force_switch_requested && observer_locked))) {
            state = STARTUP_SWITCH;
            state_elapsed = 0.0f;
            // OPENLOOP 的锁定累计只能用来准入 SWITCH。进入混合后重新
            // 统计连续健康时间，禁止沿用旧计数直接进入 CLOSED。
            lock_elapsed = 0.0f;
            invalid_elapsed = 0.0f;
            blend_weight = 0.0f;
            // 保存当前无跳变的角度偏移。SWITCH 中只衰减这一固定偏移，
            // 避免继续旋转的开环轨迹与 SMO 轨迹相互追逐。
            switch_angle_offset = wrap_pm_pi(
                openloop_theta_elec - observer_angle(smo));
            phase_error = -switch_angle_offset;
            force_switch_requested = 0U;
            switch_transition_pending = 1U;
        } else if (startup_elapsed >= config.switch_timeout) {
            fault_reason = STARTUP_FAULT_TIMEOUT;
            state = STARTUP_FAULT;
        }
        return;
    }

    case STARTUP_SWITCH: {
        state_elapsed += Ts;
        startup_elapsed += Ts;
        advance_openloop_angle();

        if (observer_direction_reversed(smo)) {
            fault_reason = STARTUP_FAULT_REVERSE;
            state = STARTUP_FAULT;
            return;
        }

        // SWITCH 允许转子随 SMO 向用户目标正常加速，但仍把 PLL 限制在
        // 启动捕获窗内；CLOSED 后解除该限制并由速度 PI 接管。
        SMO_ClampSpeed(smo,
                       openloop_speed_mech * (float)pp,
                       3.0f * config.lock_speed_tolerance * (float)pp);

        float t = state_elapsed / config.switch_duration;
        if (t > 1.0f) t = 1.0f;
        blend_weight = t * t * (3.0f - 2.0f * t); // smoothstep
        float smo_angle = observer_angle(smo);

        // output = SMO + 逐渐衰减的初始偏移：
        //   t=0 时严格等于进入 SWITCH 前的开环角度；
        //   t=1 时严格等于 SMO 角度。
        // phase_error 表示真正施加给电机的换相角与转子估计角之间的误差，
        // 其幅值会随混合权重单调收敛，不再受废弃开环轨迹漂移影响。
        float remaining_offset =
            (1.0f - blend_weight) * switch_angle_offset;
        output_angle = normalize_angle(smo_angle + remaining_offset);
        phase_error = wrap_pm_pi(smo_angle - output_angle);

        uint8_t observer_healthy = observer_healthy_during_switch(smo);

        if (observer_healthy) {
            invalid_elapsed = 0.0f;
            lock_elapsed += Ts;
        } else {
            invalid_elapsed += Ts;
            // CLOSED 准入要求连续健康；任何一次失效都重新计时。
            lock_elapsed = 0.0f;
        }

        // signal_valid 已经做过内部防抖；额外确认 20ms 后立即退回开环。
        // 从当前输出角继续可保证换相角连续，同时 OPENLOOP 会恢复确定的
        // I/F 电流，不再以零转矩停留到反电势彻底消失。
        if (invalid_elapsed >= SWITCH_INVALID_TIMEOUT) {
            openloop_theta_elec = output_angle;
            state = STARTUP_OPENLOOP;
            state_elapsed = 0.0f;
            lock_elapsed = 0.0f;
            blend_weight = 0.0f;
            switch_angle_offset = 0.0f;
            phase_error = 0.0f;
            // 将 PLL 重新种到当前无跳变的开环轨迹，清除本轮伪锁产生的
            // 速度积分；保留 SMO 电流和反电动势滤波状态以便重新捕获。
            SMO_SetAngleSpeed(smo, openloop_theta_elec,
                              openloop_speed_mech * (float)pp);
            SMO_InvalidateSignal(smo);
            return;
        }

        // 混合时间结束并不代表观测器仍然可靠。原逻辑仅检查 t，会在
        // BEMF 已失效但回退计时尚未结束时强行闭环，随后立刻撤掉 Iq
        // 并触发 observer-lost。必须确认最近一段时间持续健康。
        if (t >= 1.0f &&
            observer_healthy &&
            lock_elapsed >= config.lock_duration) {
            state = STARTUP_CLOSED;
            output_angle = smo_angle;
            switch_angle_offset = 0.0f;
            phase_error = 0.0f;
            invalid_elapsed = 0.0f;
            closed_transition_pending = 1U;
        }
        return;
    }

    case STARTUP_CLOSED:
        output_angle = observer_angle(smo);
        if (observer_direction_reversed(smo)) {
            fault_reason = STARTUP_FAULT_REVERSE;
            state = STARTUP_FAULT;
            closed_transition_pending = 0U;
            return;
        }
        // 瞬时 PLL 误差可以较大，但连续接近 ±1 意味着角度误差已接近
        // 90 度，继续输出转矩只会造成抽动。与 BEMF 有效标志共同做延时保护。
        if (SMO_IsSignalValid(smo) && fabsf(smo->pll_error) < 0.98f) {
            invalid_elapsed = 0.0f;
        } else {
            invalid_elapsed += Ts;
            if (invalid_elapsed >= config.lock_loss_timeout) {
                fault_reason = STARTUP_FAULT_OBSERVER_LOST;
                state = STARTUP_FAULT;
                closed_transition_pending = 0U;
            }
        }
        return;

    case STARTUP_FAULT:
        return;
    }
}

float Startup_GetOutputAngle(void) {
    if (state == STARTUP_CLOSED) {
        return observer_angle(AngleSrc_SMO_GetObserver());
    }
    return output_angle;
}
