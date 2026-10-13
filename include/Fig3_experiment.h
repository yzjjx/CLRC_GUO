#ifndef GUO_FIG3_EXPERIMENT_H
#define GUO_FIG3_EXPERIMENT_H

#include "LSCLRC_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace clrc_guo::fig3 {

// Panda 的关节位置范围；不适用于 FR3。
// https://frankarobotics.github.io/docs/robot_specifications.html
constexpr std::array<double, 7> Q_MIN = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr std::array<double, 7> Q_MAX = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
constexpr double POSITION_MARGIN = 0.10;
constexpr double CYCLE_TIME = 10.0;
constexpr uint64_t SYNC_MAX_PERIOD_MS = 5;
constexpr uint64_t ASYNC_MAX_PERIOD_MS = 10;

// 检查发送回调间隔；漏包不改变力矩变化率限制。
inline void check_callback_period(uint64_t period_ms, bool first, uint64_t limit_ms, double robot_time)
{
    if(!first && (period_ms == 0 || period_ms > limit_ms)) {
        throw std::runtime_error("Callback gap out of range: "
            + std::to_string(period_ms) + " ms (allowed 1.."
            + std::to_string(limit_ms) + " ms), at robot time "
            + std::to_string(robot_time) + " s. Check communication and realtime latency.");
    }
}

inline void check_control_timestamp(double t, double control_t)
{
    if(!std::isfinite(t) || t < 0.0 || !std::isfinite(control_t)
        || control_t < 0.0 || control_t > t + 1e-10) {
        throw std::runtime_error("Invalid asynchronous control result timestamp.");
    }
}

// 末次结束样本归属最后一轮，轮内时间为 10 s；轮间不重置任何状态。
inline int cycle_index(double t, int rounds)
{
    return std::min(static_cast<int>(std::floor(t / CYCLE_TIME)) + 1, rounds);
}

inline double cycle_time(double t, int rounds)
{
    return t - (cycle_index(t, rounds) - 1) * CYCLE_TIME;
}

inline JointVector position_command(double t, const JointVector& center,
                                    const JointVector& amplitude, int rounds = 1)
{
    if(t >= rounds * CYCLE_TIME) {
        return center;
    }
    const double phase = cycle_time(t, rounds);
    double sign = 0.0;
    if(phase >= 1.0 && phase < 2.0) {
        sign = -1.0;
    } else if(phase >= 5.0 && phase < 6.0) {
        sign = 1.0;
    }
    return center + sign * amplitude;
}

// 二阶参考模型的解析离散解；跨越指令切换点时分段推进。
inline void advance_reference(JointReference& ref, const JointVector& center,
                              const JointVector& amplitude, double from, double to, int rounds = 1)
{
    while(from < to) {
        double end = to;
        // 从当前轮起寻找下一切换点，漏包跨越切换点时仍精确分段。
        for(int cycle = static_cast<int>(std::floor(from / CYCLE_TIME)); cycle < rounds; cycle++) {
            if(cycle * CYCLE_TIME >= end) {
                break;
            }
            for(double phase : {1.0, 2.0, 5.0, 6.0}) {
                const double boundary = cycle * CYCLE_TIME + phase;
                if(boundary > from && boundary < end) {
                    end = boundary;
                }
            }
        }
        const double dt = end - from;
        const double decay = std::exp(-6.0 * dt);
        const JointVector qc = position_command(from, center, amplitude, rounds);
        const JointVector error = ref.q - qc;
        const JointVector b = ref.dq + 6.0 * error;
        ref.q = qc + (error + dt * b) * decay;
        ref.dq = (ref.dq - 6.0 * dt * b) * decay;
        from = end;
    }
}

inline void check_position(const JointVector& q, const JointVector& excursion = JointVector::Zero())
{
    if(!q.allFinite() || !excursion.allFinite()) {
        throw std::runtime_error("Non-finite joint position or experiment amplitude.");
    }
    for(int j = 0; j < PANDA_DOF; j++) {
        const double lower = Q_MIN[j] + POSITION_MARGIN;
        const double upper = Q_MAX[j] - POSITION_MARGIN;
        if(q(j) - std::abs(excursion(j)) <= lower || q(j) + std::abs(excursion(j)) >= upper) {
            throw std::runtime_error("Joint " + std::to_string(j + 1)
                + " has insufficient position margin: q=" + std::to_string(q(j))
                + " rad, allowed range with excursion=["
                + std::to_string(lower + std::abs(excursion(j))) + ", "
                + std::to_string(upper - std::abs(excursion(j))) + "] rad.");
        }
    }
}

inline void check_stationary(const JointVector& dq)
{
    if(!dq.allFinite() || dq.cwiseAbs().maxCoeff() > 0.05) {
        throw std::runtime_error("Robot must be stationary before starting LS-CLRC (|dq| <= 0.05 rad/s).");
    }
}

}  // namespace fig3

#endif
