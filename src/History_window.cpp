#include "History_window.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace clrc_guo {

HistoryWindow::HistoryWindow(double dt, double duration)
    : Ts(dt), duration(duration)
{
    if(!std::isfinite(dt) || dt <= 0 || !std::isfinite(duration) || duration < dt
        || duration / dt > 1000000) {
        throw std::invalid_argument("History window duration or timestep is invalid.");
    }
    // 额外一格用于窗边界上尚未完全移出的区间；控制期间不再分配。
    capacity = static_cast<int>(std::ceil(duration / dt)) + 1;
    Yf_buffer.resize(capacity, RegressorMatrix::Zero());
    tau_f_buffer.resize(capacity, JointVector::Zero());
    dt_buffer.resize(capacity, 0.0);
}

void HistoryWindow::reset()
{
    count = 0;
    next_index = 0;
    elapsed = 0.0;
    Theta.setZero();
    y.setZero();
}

void HistoryWindow::step(const RegressorMatrix& Yf, const JointVector& tau_f)
{
    step(Yf, tau_f, Ts);
}

void HistoryWindow::step(const RegressorMatrix& Yf, const JointVector& tau_f, double dt)
{
    if(!std::isfinite(dt) || dt <= 0 || dt < Ts - 1e-12 || dt > duration) {
        throw std::invalid_argument("History sample period must be between nominal dt and duration.");
    }
    // 按实际区间时长积分；窗边界落在旧区间内部时仅移出过期部分。
    double excess = std::max(0.0, elapsed + dt - duration);
    while(count > 0 && excess > 0.0) {
        const int oldest = (next_index - count + capacity) % capacity;
        const double removed = std::min(excess, dt_buffer[oldest]);
        const RegressorMatrix& old_Yf = Yf_buffer[oldest];
        Theta.noalias() -= removed * old_Yf.transpose() * old_Yf;
        y.noalias() -= removed * old_Yf.transpose() * tau_f_buffer[oldest];
        elapsed -= removed;
        excess -= removed;
        dt_buffer[oldest] -= removed;
        if(dt_buffer[oldest] <= 1e-12) {
            count--;
        }
    }
    if(count == capacity) {
        throw std::runtime_error("History window capacity exceeded.");
    }

    Yf_buffer[next_index] = Yf;
    tau_f_buffer[next_index] = tau_f;
    dt_buffer[next_index] = dt;
    count++;
    elapsed = std::min(duration, elapsed + dt);
    Theta.noalias() += dt * Yf.transpose() * Yf;
    y.noalias() += dt * Yf.transpose() * tau_f;
    Theta = (0.5 * (Theta + Theta.transpose())).eval();
    next_index = (next_index + 1) % capacity;
}

} // namespace clrc_guo
