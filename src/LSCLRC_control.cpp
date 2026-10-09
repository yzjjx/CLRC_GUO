#include "LSCLRC_control.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace clrc_guo {

PandaLSCLRC::PandaLSCLRC(const PandaRegressor& dynamics, const LSCLRCConfig& config)
    : config(config), dynamics(dynamics), history(config.dt, config.tau_d)
{
    config.validate();
    reset();
}

void PandaLSCLRC::reset()
{
    W_hat = config.initial_W_hat;
    Yf.setZero();
    tau_f.setZero();
    previous_dq.setZero();
    previous_tau.setZero();
    has_previous_sample = false;
    ie_detected = false;
    ie_time = std::numeric_limits<double>::quiet_NaN();
    best_sigma = 0.0;
    previous_time = -1.0;
    Theta_e.setZero();
    Theta_e_pinv.setZero();
    y_e.setZero();
    stored_information_rank = 0;
    stored_pinv_rank = 0;
    history.reset();
    diagnostic = LSCLRCDiagnostic();
    diagnostic.W_hat = W_hat;
}

void PandaLSCLRC::update_filter(const JointVector& q, const JointVector& dq)
{
    diagnostic.acceleration_estimate.setZero();
    diagnostic.filter_sample_valid = has_previous_sample;
    if(has_previous_sample) {
        // 差分描述刚结束的采样区间，必须匹配上一周期的施加力矩。
        diagnostic.acceleration_estimate = (dq - previous_dq) / config.dt;
        const RegressorMatrix Y_id = dynamics.inverse_regressor(
            q, dq, diagnostic.acceleration_estimate);
        const double decay = std::exp(-config.alpha * config.dt);
        const double gain = 1.0 - decay;
        Yf = decay * Yf + gain * Y_id;
        tau_f = decay * tau_f + gain * previous_tau;
    }
    previous_dq = dq;
    has_previous_sample = true;
    diagnostic.Yf = Yf;
    diagnostic.tau_f = tau_f;
}

InformationMatrix PandaLSCLRC::pseudo_inverse(const InformationMatrix& Theta, int& rank) const
{
    // 对称矩阵的奇异值为特征值的绝对值；按绝对阈值截断，不能直接求逆。
    const InformationMatrix symmetric_Theta = 0.5 * (Theta + Theta.transpose());
    Eigen::SelfAdjointEigenSolver<InformationMatrix> solver(symmetric_Theta);
    if(solver.info() != Eigen::Success) {
        throw std::runtime_error("Information matrix pseudoinverse decomposition failed.");
    }
    ParameterVector inverse_values = ParameterVector::Zero();
    rank = 0;
    for(int i = 0; i < PANDA_PARAMETERS; i++) {
        const double value = solver.eigenvalues()(i);
        if(std::abs(value) > config.pinv_tolerance) {
            inverse_values(i) = 1.0 / value;
            rank++;
        }
    }
    InformationMatrix result = solver.eigenvectors() * inverse_values.asDiagonal()
        * solver.eigenvectors().transpose();
    if(!result.allFinite()) {
        throw std::runtime_error("Information matrix pseudoinverse contains non-finite values.");
    }
    return result;
}

void PandaLSCLRC::store_history(const InformationMatrix& Theta, const ParameterVector& y)
{
    // 保存IE批及其伪逆，冻结后每周期直接使用缓存的伪逆。
    Theta_e = 0.5 * (Theta + Theta.transpose());
    y_e = y;
    Theta_e_pinv = pseudo_inverse(Theta_e, stored_pinv_rank);
    Eigen::SelfAdjointEigenSolver<InformationMatrix> solver(Theta_e, Eigen::EigenvaluesOnly);
    if(solver.info() != Eigen::Success) {
        throw std::runtime_error("Stored history eigendecomposition failed.");
    }
    const auto values = solver.eigenvalues().cwiseAbs().eval();
    const double tolerance = 1e-8 * std::max(1.0, values.maxCoeff());
    stored_information_rank = static_cast<int>((values.array() > tolerance).count());
}

void PandaLSCLRC::update_history(double t)
{
    history.step(Yf, tau_f);
    const InformationMatrix& Theta = history.information();
    const ParameterVector& y = history.torque_integral();
    Eigen::SelfAdjointEigenSolver<InformationMatrix> solver(Theta, Eigen::EigenvaluesOnly);
    if(solver.info() != Eigen::Success) {
        throw std::runtime_error("History eigendecomposition failed.");
    }
    diagnostic.sigma_min = std::max(0.0, solver.eigenvalues().minCoeff());
    const double scale = std::max(1.0, t);
    const double time_tolerance = 10 * (std::nextafter(scale,
        std::numeric_limits<double>::infinity()) - scale);

    // 1. 记录IE事件和历史批，与lsclrc2020_step.m的分支顺序一致。
    if(config.use_precomputed_batch) {
        if(!ie_detected && t + time_tolerance >= config.publication_time) {
            ie_detected = true;
            ie_time = t;
            store_history(config.batch_H, config.batch_B);
            Eigen::SelfAdjointEigenSolver<InformationMatrix> batch_solver(Theta_e, Eigen::EigenvaluesOnly);
            if(batch_solver.info() != Eigen::Success) {
                throw std::runtime_error("Batch eigendecomposition failed.");
            }
            best_sigma = 0.0;
            for(int i = 0; i < PANDA_PARAMETERS; i++) {
                if(batch_solver.eigenvalues()(i) > config.ie_threshold) {
                    best_sigma = batch_solver.eigenvalues()(i);
                    break;
                }
            }
        }
    } else {
        if(!ie_detected && history.full() && diagnostic.sigma_min >= config.ie_threshold) {
            ie_detected = true;
            ie_time = t;
            best_sigma = diagnostic.sigma_min;
            store_history(Theta, y);
        } else if(ie_detected && !config.freeze_on_ie && diagnostic.sigma_min > best_sigma) {
            best_sigma = diagnostic.sigma_min;
            store_history(Theta, y);
        }
    }

    // 2. 冻结开启且已记录IE时使用保存批，否则使用当前在线窗口。
    const bool use_frozen = ie_detected && config.freeze_on_ie;
    const InformationMatrix& Theta_use = use_frozen ? Theta_e : Theta;
    const ParameterVector& y_use = use_frozen ? y_e : y;
    diagnostic.prediction_error = y_use - Theta_use * W_hat;

    // 3. 满窗门控同时检查样本数与实际经过时间；不是必须先检测到IE。
    const bool full_duration_elapsed = t + time_tolerance >= config.tau_d;
    diagnostic.history_ready = !config.require_full_history_window
        || (history.full() && full_duration_elapsed);
    const bool has_information = Theta_use.norm() > 0;
    diagnostic.least_squares_term.setZero();
    diagnostic.pinv_rank = 0;
    if(diagnostic.history_ready && has_information) {
        if(use_frozen) {
            diagnostic.least_squares_term = Theta_e_pinv * diagnostic.prediction_error;
            diagnostic.pinv_rank = stored_pinv_rank;
        } else {
            const InformationMatrix Theta_pinv = pseudo_inverse(Theta_use, diagnostic.pinv_rank);
            diagnostic.least_squares_term = Theta_pinv * diagnostic.prediction_error;
        }
    }

    // 原MATLAB在已检测IE时报告保存批的秩，即使freeze_on_ie=false。
    diagnostic.information_rank = 0;
    if(ie_detected) {
        diagnostic.information_rank = stored_information_rank;
    } else if(diagnostic.history_ready) {
        const auto values = solver.eigenvalues().cwiseAbs().eval();
        const double tolerance = 1e-8 * std::max(1.0, values.maxCoeff());
        diagnostic.information_rank = static_cast<int>((values.array() > tolerance).count());
    }
    diagnostic.ie_detected = ie_detected;
    diagnostic.ie_time = ie_time;
    diagnostic.best_sigma = best_sigma;
    diagnostic.history_fraction = history.fraction();
    diagnostic.history_update_active = diagnostic.history_ready && has_information;
    diagnostic.using_frozen_history = use_frozen;
}

void PandaLSCLRC::update_parameters(const ParameterVector& raw_direction)
{
    // 对应project_parameter_step.m：先投影原方向，再左乘Gamma。
    ParameterVector projected = raw_direction;
    const double w_norm = W_hat.norm();
    const double bound = config.parameter_bound;
    if(std::isfinite(bound)) {
        const double scale = std::max(1.0, bound);
        const double tolerance = 100 * (std::nextafter(scale,
            std::numeric_limits<double>::infinity()) - scale);
        const bool inside = w_norm < bound - tolerance;
        const bool on_boundary_inward = std::abs(w_norm - bound) <= tolerance
            && W_hat.dot(raw_direction) <= 0;
        if(!inside && !on_boundary_inward && w_norm > 0) {
            projected -= W_hat * (W_hat.dot(raw_direction) / W_hat.squaredNorm());
        }
    }
    diagnostic.W_dot = config.Gamma * projected;
    W_hat += config.dt * diagnostic.W_dot;
    if(!W_hat.allFinite()) {
        throw std::runtime_error("Parameter update contains non-finite values.");
    }
    if(std::isfinite(bound) && W_hat.norm() > bound) {
        W_hat *= bound / W_hat.norm();
    }
    diagnostic.W_hat = W_hat;
}

JointVector PandaLSCLRC::compute_control(
    double t,
    const JointVector& q,
    const JointVector& dq,
    const JointReference& reference)
{
    if(!std::isfinite(t) || t < 0 || !q.allFinite() || !dq.allFinite()
        || !reference.q.allFinite() || !reference.dq.allFinite() || !reference.ddq.allFinite()) {
        throw std::invalid_argument("LS-CLRC measurement or reference is invalid.");
    }
    if(previous_time >= 0 && std::abs(t - previous_time - config.dt) > 1e-8 * std::max(1.0, t)) {
        throw std::invalid_argument("LS-CLRC requires a fixed control timestep; call reset before restarting.");
    }

    // 1. 论文式(5)：tau = e1 + K2*e2 + Y*W_hat，使用更新前参数。
    diagnostic.e = reference.q - q;
    diagnostic.filtered_error = reference.dq - dq + config.K1 * diagnostic.e;
    const JointVector v = reference.dq + config.K1 * diagnostic.e;
    const JointVector dv = reference.ddq + config.K1 * (reference.dq - dq);
    const RegressorMatrix Y = dynamics.tracking_regressor(q, dq, v, dv);
    diagnostic.predicted_torque = Y * W_hat;
    diagnostic.raw_torque = diagnostic.e + config.K2 * diagnostic.filtered_error
        + diagnostic.predicted_torque;
    if(!diagnostic.raw_torque.allFinite()) {
        throw std::runtime_error("Control torque contains non-finite values.");
    }

    JointVector tau;
    for(int joint = 0; joint < PANDA_DOF; joint++) {
        const double limited = std::clamp(diagnostic.raw_torque(joint),
            -config.torque_limit(joint), config.torque_limit(joint));
        if(std::isfinite(config.torque_rate_limit(joint))) {
            const double delta = config.torque_rate_limit(joint) * config.dt;
            tau(joint) = std::clamp(limited, previous_tau(joint) - delta, previous_tau(joint) + delta);
        } else {
            tau(joint) = limited;
        }
    }

    // 2. 共同滤波 -> 在线历史积分 -> 选择历史批 -> 最小二乘调制。
    update_filter(q, dq);
    update_history(t);

    // 3. 论文式(12)：Gamma*Proj(Y'*e2 + Kw*Theta_use^+*epsilon)。
    const ParameterVector direct_term = Y.transpose() * diagnostic.filtered_error;
    const ParameterVector history_term = config.Kw * diagnostic.least_squares_term;
    diagnostic.direct_learning_term_norm = direct_term.norm();
    diagnostic.history_learning_term_norm = history_term.norm();
    update_parameters(direct_term + history_term);
    previous_tau = tau;
    previous_time = t;
    return tau;
}

void PandaLSCLRC::set_applied_torque(const JointVector& tau)
{
    if(!tau.allFinite() || (tau.cwiseAbs().array() > config.torque_limit.array() + 1e-10).any()) {
        throw std::invalid_argument("Applied torque is non-finite or exceeds configured limits.");
    }
    previous_tau = tau;
}

} // namespace clrc_guo
