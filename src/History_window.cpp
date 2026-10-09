#include "History_window.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace clrc_guo {

HistoryWindow::HistoryWindow(double dt, double duration)
    : Ts(dt)
{
    if(!std::isfinite(dt) || dt <= 0 || !std::isfinite(duration) || duration <= 0
        || duration / dt > 1000000) {
        throw std::invalid_argument("History window duration or timestep is invalid.");
    }
    capacity = std::max(1, static_cast<int>(std::lround(duration / dt)));
    Yf_buffer.resize(capacity, RegressorMatrix::Zero());
    tau_f_buffer.resize(capacity, JointVector::Zero());
}

void HistoryWindow::reset()
{
    count = 0;
    next_index = 0;
    Theta.setZero();
    y.setZero();
}

void HistoryWindow::step(const RegressorMatrix& Yf, const JointVector& tau_f)
{
    // 固定周期环形缓存：满窗后先减去最旧样本，再加上当前样本。
    if(count == capacity) {
        const RegressorMatrix& old_Yf = Yf_buffer[next_index];
        const JointVector& old_tau_f = tau_f_buffer[next_index];
        Theta.noalias() -= Ts * old_Yf.transpose() * old_Yf;
        y.noalias() -= Ts * old_Yf.transpose() * old_tau_f;
    } else {
        count++;
    }

    Yf_buffer[next_index] = Yf;
    tau_f_buffer[next_index] = tau_f;
    Theta.noalias() += Ts * Yf.transpose() * Yf;
    y.noalias() += Ts * Yf.transpose() * tau_f;
    Theta = (0.5 * (Theta + Theta.transpose())).eval();
    next_index = (next_index + 1) % capacity;
}

} // namespace clrc_guo
