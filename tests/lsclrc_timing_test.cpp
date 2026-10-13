#include "LSCLRC_control.h"
#include "Fig3_experiment.h"

#include <algorithm>
#include <chrono>
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

void check_history()
{
    HistoryWindow history(0.001, 0.0035);
    RegressorMatrix Y = RegressorMatrix::Zero();
    JointVector tau = JointVector::Zero();
    Y(0, 0) = 1.0;
    tau(0) = 2.0;
    history.step(Y, tau, 0.002);
    require(!history.full(), "History filled before elapsed window duration.");
    require(std::abs(history.fraction() - 2.0 / 3.5) < 1e-12, "Incorrect history elapsed fraction.");
    Y(0, 0) = 2.0;
    tau(0) = 3.0;
    history.step(Y, tau, 0.002);
    require(history.full(), "History does not fill by elapsed time.");
    require(std::abs(history.information()(0, 0) - 0.0095) < 1e-12, "Incorrect partial oldest information integral.");
    require(std::abs(history.torque_integral()(0) - 0.015) < 1e-12, "Incorrect partial oldest torque integral.");
    Y(0, 0) = 3.0;
    tau(0) = 4.0;
    history.step(Y, tau, 0.003);
    require(std::abs(history.information()(0, 0) - 0.029) < 1e-12, "Incorrect removal across multiple old intervals.");
    require(std::abs(history.torque_integral()(0) - 0.039) < 1e-12, "Incorrect weighted torque integral.");
    require_rejected([&] { history.step(Y, tau, 0.0005); }, "Accepted period below preallocated minimum.");
    history.reset();
    require(!history.full() && history.fraction() == 0.0 && history.information().isZero(), "History reset failed.");
    // 多次环绕，同时与明确的最近 3.5 ms 区间积分比较。
    for(int i = 0; i < 1000; i++) {
        history.step(Y, tau);
    }
    require(std::abs(history.information()(0, 0) - 0.0035 * 9.0) < 1e-11, "Ring-buffer wrapping changed window duration.");
}

void check_controller()
{
    PandaRegressor dynamics;
    LSCLRCConfig config;
    config.torque_limit.setConstant(87.0);
    config.torque_rate_limit.setConstant(800.0);
    PandaLSCLRC controller(dynamics, config);
    JointVector q;
    q << 0.0, -0.5, 0.0, -1.8, 0.0, 1.5, 0.7;
    JointReference ref;
    ref.q = q;
    const JointVector zero = JointVector::Zero();
    controller.compute_control(0.0, q, zero, ref);
    const JointVector applied = JointVector::Constant(0.5);
    controller.set_applied_torque(applied);
    const double dt = 0.003;
    const JointVector dq = JointVector::Constant(dt);
    const ParameterVector previous_w = controller.parameters();
    const JointVector tau = controller.compute_control(dt, q, dq, ref, dt);
    const auto& d = controller.diagnostics();
    require(d.acceleration_estimate.isApprox(JointVector::Ones(), 1e-12), "Acceleration used nominal rather than actual dt.");
    const double gain = 1.0 - std::exp(-config.alpha * dt);
    require(d.Yf.isApprox(gain * dynamics.inverse_regressor(q, dq, JointVector::Ones()), 1e-12), "Regressor filter used incorrect dt.");
    require(d.tau_f.isApprox(gain * applied, 1e-12), "Torque filter used incorrect period or input alignment.");
    require(controller.parameters().isApprox(previous_w + dt * d.W_dot, 1e-12), "Parameter Euler update used incorrect dt.");
    require(((tau - applied).cwiseAbs().array() <= config.torque_rate_limit.array() * dt + 1e-12).all(), "Controller torque slew exceeds actual interval bound.");
    require_rejected([&] { controller.compute_control(0.004, q, dq, ref, 0.002); }, "Accepted inconsistent timestamp and sample period.");
    require_rejected([&] { controller.compute_control(0.0035, q, dq, ref, 0.0005); }, "Accepted too-short period.");

    PandaLSCLRC fixed(dynamics, config);
    PandaLSCLRC explicit_dt(dynamics, config);
    for(int i = 0; i < 20; i++) {
        require(fixed.compute_control(i * config.dt, q, dq, ref).isApprox(
            explicit_dt.compute_control(i * config.dt, q, dq, ref, config.dt), 1e-12),
            "Explicit nominal period changed fixed-period behavior.");
    }
}

void check_reference_and_guards()
{
    JointVector center;
    center << 0.0, -0.5, 0.0, -1.8, 0.0, 1.5, 0.7;
    const JointVector amplitude = JointVector::Constant(0.1);
    JointReference ref;
    ref.q = center;
    fig3::advance_reference(ref, center, amplitude, 0.999, 1.002);
    const double decay = std::exp(-6.0 * 0.002);
    require(ref.q.isApprox(center - amplitude + amplitude * (1.0 + 6.0 * 0.002) * decay, 1e-12),
            "Reference applied step command before its switching time.");
    require(ref.dq.isApprox(-36.0 * 0.002 * decay * amplitude, 1e-12), "Reference velocity across boundary is incorrect.");
    JointReference split = ref;
    fig3::advance_reference(ref, center, amplitude, 1.002, 6.003);
    for(const auto interval : {std::array<double, 2>{1.002, 2.0}, {2.0, 5.0}, {5.0, 6.0}, {6.0, 6.003}}) {
        fig3::advance_reference(split, center, amplitude, interval[0], interval[1]);
    }
    require(ref.q.isApprox(split.q, 1e-12) && ref.dq.isApprox(split.dq, 1e-12), "Multiple reference boundaries were skipped.");
    fig3::check_position(center, amplitude);
    fig3::check_stationary(JointVector::Zero());
    JointVector near_limit = center;
    near_limit(3) = fig3::Q_MAX[3] - 0.15;
    require_rejected([&] { fig3::check_position(near_limit, amplitude); }, "Accepted pulse trajectory outside position margin.");
    near_limit(3) = fig3::Q_MAX[3] - 0.05;
    require_rejected([&] { fig3::check_position(near_limit); }, "Accepted measurement close to hard joint limit.");
    require_rejected([&] { fig3::check_stationary(JointVector::Constant(0.1)); }, "Accepted moving robot at startup.");
}

void check_repeated_reference()
{
    const JointVector center = JointVector::Constant(0.3);
    const JointVector amplitude = JointVector::Constant(0.1);
    for(int cycle = 0; cycle < 10; cycle++) {
        const double start = cycle * fig3::CYCLE_TIME;
        for(double phase : {0.0, 0.999, 1.0, 1.999, 2.0, 4.999, 5.0, 5.999, 6.0, 9.999}) {
            require(fig3::position_command(start + phase, center, amplitude, 10).isApprox(
                fig3::position_command(phase, center, amplitude), 1e-12),
                "Repeated pulse differs from the original 10-second command.");
            require(fig3::cycle_index(start + phase, 10) == cycle + 1,
                "Incorrect repeated experiment round number.");
        }
    }
    require(fig3::cycle_index(100.0, 10) == 10 && fig3::cycle_time(100.0, 10) == 10.0,
        "Terminal sample incorrectly starts an eleventh round.");
    require(fig3::position_command(101.2, center, amplitude, 10).isApprox(center, 1e-12),
        "Excitation continues beyond the requested ten rounds.");
    require(fig3::position_command(11.2, center, amplitude).isApprox(center, 1e-12),
        "Single-round experiment unexpectedly repeats its pulse.");

    // 逐轮分段与一次跨越多轮应得到同一参考状态；边界处不重置 qd/dqd。
    JointReference whole;
    whole.q = center;
    whole.dq = JointVector::Constant(0.07);
    JointReference split = whole;
    fig3::advance_reference(whole, center, amplitude, 0.0, 100.003, 10);
    for(int cycle = 0; cycle < 10; cycle++) {
        const double start = cycle * fig3::CYCLE_TIME;
        double from = start;
        for(double phase : {1.0, 2.0, 5.0, 6.0, 10.0}) {
            fig3::advance_reference(split, center, amplitude, from, start + phase, 10);
            from = start + phase;
        }
    }
    fig3::advance_reference(split, center, amplitude, 100.0, 100.003, 10);
    // 末段速度接近零，使用绝对误差，避免相对误差放大舍入误差。
    require((whole.q - split.q).cwiseAbs().maxCoeff() < 1e-12
        && (whole.dq - split.dq).cwiseAbs().maxCoeff() < 1e-12,
        "Reference integration skipped a repeated pulse or reset at a round boundary.");

    // 第二轮负脉冲的漏包跨界：前 1 ms 保持，后 2 ms 为负阶跃。
    JointReference gap;
    gap.q = center;
    fig3::advance_reference(gap, center, amplitude, 10.999, 11.002, 10);
    const double decay = std::exp(-6.0 * 0.002);
    require(gap.q.isApprox(center - amplitude + amplitude * (1.0 + 6.0 * 0.002) * decay, 1e-12)
        && gap.dq.isApprox(-36.0 * 0.002 * decay * amplitude, 1e-12),
        "Missed callback across a repeated pulse switching time is incorrect.");
}

void check_ten_second_run(int rounds = 1)
{
    PandaRegressor dynamics;
    LSCLRCConfig config;
    config.initial_W_hat = PandaRegressor::nominal_parameters();
    PandaLSCLRC controller(dynamics, config);
    JointVector center;
    center << 0.0, -0.5, 0.0, -1.8, 0.0, 1.5, 0.7;
    const JointVector zero = JointVector::Zero();
    const JointVector nominal_hold = dynamics.inverse_regressor(center, zero, zero) * config.initial_W_hat;
    controller.set_applied_torque(nominal_hold);
    JointReference ref;
    ref.q = center;
    const JointVector amplitude = JointVector::Constant(0.08);
    uint64_t elapsed_ms = 0;
    std::size_t count = 0;
    std::size_t gaps = 0;
    double max_us = 0.0;
    double previous_best_sigma = 0.0;
    const auto start = std::chrono::steady_clock::now();
    while(true) {
        const uint64_t period_ms = count == 0 ? 0 : (count % 37 == 0 ? 5 : (count % 13 == 0 ? 2 : 1));
        if(period_ms > 1) {
            gaps++;
        }
        const double previous_t = elapsed_ms * config.dt;
        elapsed_ms += period_ms;
        const double t = elapsed_ms * config.dt;
        fig3::advance_reference(ref, center, amplitude, previous_t, t, rounds);
        ref.ddq = 36.0 * (fig3::position_command(t, center, amplitude, rounds) - ref.q) - 12.0 * ref.dq;
        const auto begin = std::chrono::steady_clock::now();
        const JointVector tau = controller.compute_control(t, ref.q, ref.dq, ref,
            count == 0 ? config.dt : period_ms * config.dt);
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin).count();
        max_us = std::max(max_us, us);
        controller.set_applied_torque(tau);
        require(tau.allFinite() && controller.parameters().allFinite(), "Non-finite output after missed callbacks.");
        require(controller.parameters().norm() <= config.parameter_bound + 1e-10, "Parameters escaped configured bound.");
        require(controller.diagnostics().best_sigma >= previous_best_sigma,
            "Repeated experiment lost its selected historical batch.");
        previous_best_sigma = controller.diagnostics().best_sigma;
        if(t >= config.tau_d) {
            const auto& d = controller.diagnostics();
            const double selected_sigma = d.using_frozen_history ? d.best_sigma : d.sigma_min;
            require(d.history_update_active == (selected_sigma >= config.history_activation_threshold),
                "Repeated history learning ignored the selected batch excitation threshold.");
        }
        if(count == 0) {
            require((tau - nominal_hold).norm() < 1e-10, "Nominal initialization did not preserve holding torque at startup.");
        }
        count++;
        if(elapsed_ms >= static_cast<uint64_t>(rounds) * 10000) {
            break;
        }
    }
    require(controller.diagnostics().history_fraction > 1.0 - 1e-10, "History did not retain full 3-second window with missed samples.");
    const double total_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    std::cout << "Offline " << rounds * 10 << " s replay: " << count << " callbacks, " << gaps << " gaps, mean loop="
              << total_us / count << " us, max controller=" << max_us << " us\n";
}

int main()
{
    try {
        check_history();
        check_controller();
        check_reference_and_guards();
        check_repeated_reference();
        check_ten_second_run();
        std::cout << "LS-CLRC timing and startup checks passed.\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
