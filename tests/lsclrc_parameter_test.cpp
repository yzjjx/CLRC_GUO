#include "LSCLRC_control.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace clrc_guo;

void require(bool condition, const char* message)
{
    if(!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Function>
void require_rejected(Function function, const char* message)
{
    bool rejected = false;
    try {
        function();
    } catch(const std::exception&) {
        rejected = true;
    }
    require(rejected, message);
}

JointReference stationary_reference()
{
    JointReference ref;
    ref.q << 0.0, -0.5, 0.0, -1.8, 0.0, 1.5, 0.7;
    return ref;
}

void check_control_without_direct_learning()
{
    PandaRegressor dynamics;
    LSCLRCConfig config;
    config.initial_W_hat = PandaRegressor::nominal_parameters();
    PandaLSCLRC controller(dynamics, config);
    JointReference ref = stationary_reference();
    const JointVector q = ref.q - JointVector::Constant(0.01);
    const JointVector dq = JointVector::Constant(0.02);
    const JointVector e1 = ref.q - q;
    const JointVector e2 = ref.dq - dq + config.K1 * e1;
    const RegressorMatrix Y = dynamics.tracking_regressor(q, dq,
        ref.dq + config.K1 * e1, ref.ddq + config.K1 * (ref.dq - dq));
    const JointVector expected = e1 + config.K2 * e2 + Y * config.initial_W_hat;
    require((Y.transpose() * e2).norm() > 1e-6,
        "Direct-learning test did not provide a nonzero tracking learning direction.");
    const JointVector tau = controller.compute_control(0.0, q, dq, ref);
    require(tau.isApprox(expected, 1e-12), "Guo control law or pre-update estimate changed.");
    double t = 0.0;
    for(int k = 0; k < 10; k++) {
        if(k > 0) {
            const double dt = k % 2 == 0 ? 0.003 : config.dt;
            t += dt;
            controller.compute_control(t, q, dq, ref, dt);
        }
        const auto& d = controller.diagnostics();
        require(!d.history_update_active, "History activated before full window.");
        require(controller.parameters() == config.initial_W_hat && d.W_dot.isZero()
            && d.direct_learning_term_norm == 0.0 && d.history_learning_term_norm == 0.0,
            "Tracking error changed parameters while composite learning was inactive.");
    }
}

void check_composite_learning_with_tracking_error()
{
    PandaRegressor dynamics;
    LSCLRCConfig config;
    JointVector k2;
    k2 << 15, 15, 15, 15, 6, 6, 3.75;
    require(config.K1 == 6.0 * JointMatrix::Identity()
        && config.K2 == JointMatrix(k2.asDiagonal())
        && config.Gamma == 0.3 * InformationMatrix::Identity()
        && config.Kw == 2.5 * InformationMatrix::Identity(),
        "Tracking or composite learning gains changed.");
    config.initial_W_hat = PandaRegressor::nominal_parameters();
    config.require_full_history_window = false;
    config.use_precomputed_batch = true;
    config.publication_time = 0.0;
    config.batch_H = 2.0 * InformationMatrix::Identity();
    const ParameterVector target = config.initial_W_hat + ParameterVector::Constant(0.01);
    config.batch_B = config.batch_H * target;
    PandaLSCLRC controller(dynamics, config);
    const JointReference ref = stationary_reference();
    const JointVector q = ref.q - JointVector::Constant(0.01);
    const JointVector dq = JointVector::Constant(0.02);
    double t = 0.0;
    for(int k = 0; k < 10; k++) {
        const double dt = k % 2 == 0 ? config.dt : 0.003;
        if(k > 0) {
            t += dt;
        }
        const ParameterVector previous_w = controller.parameters();
        const ParameterVector expected_dot = config.Gamma * config.Kw * (target - previous_w);
        controller.compute_control(t, q, dq, ref, dt);
        const auto& d = controller.diagnostics();
        require(!d.filtered_error.isZero() && d.history_update_active
            && d.history_learning_term_norm > 0.0 && d.direct_learning_term_norm == 0.0,
            "Nonzero tracking error disabled composite learning or restored direct learning.");
        require(d.W_dot.isApprox(expected_dot, 1e-12)
            && controller.parameters().isApprox(previous_w + dt * expected_dot, 1e-12),
            "Parameter update differs from composite-only learning with the actual interval.");
    }
}

void check_rank_deficient_pseudoinverse()
{
    PandaRegressor dynamics;
    LSCLRCConfig config;
    config.history_activation_threshold = 0.0; // 显式验证原有低秩伪逆分支
    config.require_full_history_window = false;
    config.use_precomputed_batch = true;
    config.publication_time = 0.0;
    config.batch_H(0, 0) = 2074.28;
    config.batch_H(1, 1) = 2.0;
    config.batch_H(2, 2) = 1e-12; // 应被pinv的绝对阈值截断
    ParameterVector target = ParameterVector::Zero();
    target(0) = 0.2;
    target(1) = -0.1;
    target(2) = 0.3;
    config.batch_B = config.batch_H * target;
    config.batch_B(3) = 1.0; // 零信息方向的epsilon不能产生更新
    PandaLSCLRC controller(dynamics, config);
    const JointReference ref = stationary_reference();
    double t = 0.0;
    double expected0 = 0.0;
    double expected1 = 0.0;
    for(int k = 0; k < 400; k++) {
        const double dt = k == 0 ? config.dt : (k % 2 == 0 ? 0.005 : 0.015);
        if(k > 0) {
            t += dt;
        }
        controller.compute_control(t, ref.q, JointVector::Zero(), ref, dt);
        expected0 += dt * 0.3 * 2.5 * (target(0) - expected0);
        expected1 += dt * 0.3 * 2.5 * (target(1) - expected1);
        const ParameterVector& w = controller.parameters();
        require(std::abs(w(0) - expected0) < 1e-12 && std::abs(w(1) - expected1) < 1e-12,
            "Historical learning scales with eigenvalue instead of least-squares modulation.");
        require(w.tail<PANDA_PARAMETERS - 2>().isZero(1e-12),
            "Truncated or null-space direction produced historical learning.");
        require(controller.diagnostics().pinv_rank == 2 && controller.diagnostics().using_frozen_history,
            "Pseudoinverse rank or frozen-batch selection is incorrect.");
    }
    require(std::abs(expected0 - target(0)) < 0.011, "Least-squares historical learning did not reduce error.");
    controller.reset();
    require(controller.parameters().isZero() && !controller.diagnostics().ie_detected
        && controller.diagnostics().pinv_rank == 0, "Reset retained parameters or frozen history.");
}

void check_history_gate_and_freeze_switch()
{
    PandaRegressor dynamics;
    const JointReference ref = stationary_reference();
    LSCLRCConfig config;
    config.tau_d = 0.003;
    config.use_precomputed_batch = true;
    config.publication_time = 0.0;
    config.batch_H = InformationMatrix::Identity();
    config.batch_B = ParameterVector::Ones();
    PandaLSCLRC controller(dynamics, config);
    controller.compute_control(0.0, ref.q, JointVector::Zero(), ref);
    require(controller.diagnostics().ie_detected && !controller.diagnostics().history_ready
        && controller.parameters().isZero(), "Batch publication bypassed full-window gate.");
    controller.compute_control(0.002, ref.q, JointVector::Zero(), ref, 0.002);
    require(!controller.diagnostics().history_update_active, "Sample fullness bypassed elapsed-time gate.");
    controller.compute_control(0.003, ref.q, JointVector::Zero(), ref);
    require(controller.diagnostics().history_update_active && controller.parameters().norm() > 0.0,
        "Full-window gate did not enable frozen history.");

    config.freeze_on_ie = false;
    config.require_full_history_window = false;
    PandaLSCLRC live(dynamics, config);
    live.compute_control(0.0, ref.q, JointVector::Zero(), ref);
    require(live.diagnostics().ie_detected && !live.diagnostics().using_frozen_history
        && !live.diagnostics().history_update_active && live.parameters().isZero(),
        "Freeze-off branch used the external batch instead of empty online information.");

    // 非零、低秩在线窗口可学习，不要求先满足完整IE。
    LSCLRCConfig partial;
    partial.history_activation_threshold = 0.0;
    partial.require_full_history_window = false;
    PandaLSCLRC online(dynamics, partial);
    online.compute_control(0.0, ref.q, JointVector::Zero(), ref);
    online.set_applied_torque(JointVector::Ones());
    online.compute_control(partial.dt, ref.q, JointVector::Zero(), ref);
    require(online.diagnostics().history_update_active && !online.diagnostics().ie_detected
        && online.diagnostics().pinv_rank > 0 && online.diagnostics().pinv_rank < PANDA_PARAMETERS,
        "Rank-deficient online window failed to provide partial history learning.");
}

void check_projection_and_configuration()
{
    PandaRegressor dynamics;
    const JointReference ref = stationary_reference();
    LSCLRCConfig config;
    config.history_activation_threshold = 0.0; // 本测试隔离参数投影，使用秩亏批
    config.parameter_bound = 0.01;
    config.initial_W_hat(0) = config.parameter_bound;
    config.require_full_history_window = false;
    config.use_precomputed_batch = true;
    config.publication_time = 0.0;
    config.batch_H = InformationMatrix::Identity();
    config.batch_B(0) = 1.0;
    PandaLSCLRC boundary(dynamics, config);
    boundary.compute_control(0.0, ref.q, JointVector::Zero(), ref);
    require(boundary.diagnostics().W_dot.isZero(1e-12)
        && boundary.parameters().isApprox(config.initial_W_hat, 1e-12),
        "Projection failed to remove the outward boundary direction.");
    config.batch_B(0) = -1.0;
    config.batch_B(1) = 1.0;
    config.Gamma(0, 1) = config.Gamma(1, 0) = 0.05;
    PandaLSCLRC inward(dynamics, config);
    inward.compute_control(0.0, ref.q, JointVector::Zero(), ref);
    const ParameterVector expected_dot = config.Gamma * config.Kw * (config.batch_B - config.initial_W_hat);
    require(inward.diagnostics().W_dot.isApprox(expected_dot, 1e-12),
        "Projection rejected inward motion or changed Gamma multiplication order.");
    config.Gamma *= 100000.0;
    PandaLSCLRC clipped(dynamics, config);
    clipped.compute_control(0.0, ref.q, JointVector::Zero(), ref);
    require(std::abs(clipped.parameters().norm() - config.parameter_bound) < 1e-12,
        "Euler update escaped the parameter sphere.");
    config.tau_d = config.dt / 2.0;
    require_rejected([&] { config.validate(); }, "Accepted a history duration below the nominal period.");
    LSCLRCConfig invalid;
    invalid.Kw(0, 0) = -1.0;
    require_rejected([&] { invalid.validate(); }, "Accepted non-positive learning gain.");
    invalid = LSCLRCConfig();
    invalid.batch_H(0, 0) = -1.0;
    invalid.use_precomputed_batch = true;
    require_rejected([&] { invalid.validate(); }, "Accepted negative historical information.");
    invalid = LSCLRCConfig();
    invalid.history_activation_threshold = -1.0;
    require_rejected([&] { invalid.validate(); }, "Accepted negative history activation threshold.");
    invalid.history_activation_threshold = std::numeric_limits<double>::quiet_NaN();
    require_rejected([&] { invalid.validate(); }, "Accepted NaN history activation threshold.");
    invalid.history_activation_threshold = std::numeric_limits<double>::infinity();
    require_rejected([&] { invalid.validate(); }, "Accepted infinite history activation threshold.");
}

void check_history_excitation_threshold()
{
    PandaRegressor dynamics;
    const JointReference ref = stationary_reference();
    LSCLRCConfig config;
    config.require_full_history_window = false;
    config.use_precomputed_batch = true;
    config.publication_time = 0.0;
    config.initial_W_hat = PandaRegressor::nominal_parameters();
    const double threshold = config.history_activation_threshold;
    require(threshold == 1e-2 && config.ie_threshold == 1e-2,
        "Conservative history thresholds changed unexpectedly.");

    // 复现当前冻结批的极端条件数与异常偏置方向，发布时间不能绕过新门控。
    config.batch_H = InformationMatrix::Identity();
    config.batch_H(58, 58) = 1e-6;
    config.batch_H(0, 0) = 2356.85;
    ParameterVector target = config.initial_W_hat;
    target(58) = 106.6;
    config.batch_B = config.batch_H * target;
    PandaLSCLRC weak(dynamics, config);
    for(int k = 0; k < 5; k++) {
        weak.compute_control(k * config.dt, ref.q, JointVector::Zero(), ref);
        const auto& d = weak.diagnostics();
        require(d.ie_detected && d.using_frozen_history && d.history_ready,
            "Weak batch did not reach the excitation-gate test.");
        require(d.best_sigma > threshold && !d.prediction_error.isZero(),
            "Weak batch did not distinguish best_sigma from its minimum eigenvalue.");
        require(!d.history_update_active && d.pinv_rank == 0
            && d.least_squares_term.isZero() && d.history_learning_term_norm == 0.0
            && weak.parameters().isApprox(config.initial_W_hat, 1e-12),
            "Ill-conditioned frozen history bypassed the activation threshold.");
    }

    // 门槛以下关闭，等于及高于门槛允许学习；当前在线窗为空也不能误关合格冻结批。
    for(double minimum : {0.5 * threshold, threshold, 2.0 * threshold}) {
        config.batch_H = minimum * InformationMatrix::Identity();
        target = config.initial_W_hat;
        target(58) += 0.2;
        config.batch_B = config.batch_H * target;
        PandaLSCLRC controller(dynamics, config);
        controller.compute_control(0.0, ref.q, JointVector::Zero(), ref);
        const auto& d = controller.diagnostics();
        const bool expected_active = minimum >= threshold;
        require(d.sigma_min == 0.0 && d.history_update_active == expected_active,
            "History activation boundary or frozen-batch eigenvalue selection is incorrect.");
        const ParameterVector expected = expected_active
            ? ParameterVector(config.initial_W_hat + config.dt * config.Gamma * config.Kw
                * (target - config.initial_W_hat)) : config.initial_W_hat;
        require(controller.parameters().isApprox(expected, 1e-12)
            && d.pinv_rank == (expected_active ? PANDA_PARAMETERS : 0),
            "Excitation gating changed the permitted learning law or inactive diagnostics.");
        controller.reset();
        require(!controller.diagnostics().history_update_active
            && !controller.diagnostics().using_frozen_history,
            "Reset retained the excitation-gated history state.");
    }

    // 关闭满窗口要求只跳过时间门控，仍不能让低秩在线信息进入默认历史学习。
    LSCLRCConfig online_config;
    online_config.require_full_history_window = false;
    PandaLSCLRC online(dynamics, online_config);
    online.compute_control(0.0, ref.q, JointVector::Zero(), ref);
    online.set_applied_torque(JointVector::Ones());
    online.compute_control(online_config.dt, ref.q, JointVector::Zero(), ref);
    require(online.diagnostics().history_ready && !online.diagnostics().Yf.isZero()
        && !online.diagnostics().history_update_active
        && online.diagnostics().least_squares_term.isZero(),
        "Online low-rank history bypassed the default excitation gate.");
}

int main()
{
    try {
        check_control_without_direct_learning();
        check_composite_learning_with_tracking_error();
        check_rank_deficient_pseudoinverse();
        check_history_gate_and_freeze_switch();
        check_projection_and_configuration();
        check_history_excitation_threshold();
        std::cout << "Guo control, least-squares learning, gating and projection checks passed.\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
