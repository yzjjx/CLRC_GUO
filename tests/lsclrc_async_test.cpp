#include "LSCLRC_async.h"
#include "Fig3_experiment.h"

#include <iostream>
#include <limits>

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

void check_sender_timing()
{
    // 复现日志中的 0.782 -> 0.789 s 间隔，最近结果为 0.780 s。
    fig3::check_callback_period(0, true, fig3::ASYNC_MAX_PERIOD_MS, 0.0);
    fig3::check_callback_period(7, false, fig3::ASYNC_MAX_PERIOD_MS, 0.789);
    fig3::check_control_timestamp(0.789, 0.780);
    require_rejected([&] {
        fig3::check_callback_period(7, false, fig3::SYNC_MAX_PERIOD_MS, 0.789);
    }, "Synchronous entry unexpectedly accepted the extended gap.");
    fig3::check_callback_period(10, false, fig3::ASYNC_MAX_PERIOD_MS, 0.792);
    require_rejected([&] {
        fig3::check_callback_period(11, false, fig3::ASYNC_MAX_PERIOD_MS, 0.793);
    }, "Sender accepted a callback gap exceeding 10 ms.");
    require_rejected([&] {
        fig3::check_callback_period(0, false, fig3::ASYNC_MAX_PERIOD_MS, 0.789);
    }, "Sender accepted zero elapsed time after startup.");
    // 超过原来的四个计算周期仍保持旧结果，结果年龄不再触发停止。
    fig3::check_control_timestamp(0.800, 0.780);
    fig3::check_control_timestamp(0.801, 0.780);
    fig3::check_control_timestamp(1.780, 0.780);
    require_rejected([&] {
        fig3::check_control_timestamp(0.789, 0.790);
    }, "Sender accepted a calculation result from a future timestamp.");
    require_rejected([&] {
        fig3::check_control_timestamp(0.789, std::numeric_limits<double>::quiet_NaN());
    }, "Sender accepted a non-finite result timestamp.");
    require_rejected([&] {
        fig3::check_control_timestamp(std::numeric_limits<double>::infinity(), 0.780);
    }, "Sender accepted a non-finite robot timestamp.");
    require_rejected([&] { fig3::check_control_timestamp(0.789, -0.001); },
        "Sender accepted a negative result timestamp.");
    require_rejected([&] { fig3::check_control_timestamp(-0.001, 0.0); },
        "Sender accepted a negative robot timestamp.");
}

void check_queue()
{
    SpscQueue<LSCLRCInput> queue;
    LSCLRCInput input;
    LSCLRCInput received;
    require(!queue.try_pop(received), "Empty queue returned a state.");
    for(int i = 0; i < 3; i++) {
        input.t = i;
        require(queue.try_push(input), "Queue filled before its declared usable capacity.");
    }
    input.t = 99.0;
    require(!queue.try_push(input), "Full queue overwrote an unread state.");
    for(int i = 0; i < 3; i++) {
        require(queue.try_pop(received) && received.t == i, "Queue changed state order.");
    }

    // 实际并发传输带多字段的状态，验证发布后看不到半写入的数据。
    constexpr int count = 20000;
    std::atomic<bool> valid{true};
    std::thread consumer([&] {
        LSCLRCInput value;
        for(int i = 0; i < count; i++) {
            while(!queue.try_pop(value)) {
                std::this_thread::yield();
            }
            if(value.t != i || !value.q.isConstant(i) || !value.dq.isConstant(-i)
                || !value.torque_integral.isConstant(2.0 * i)) {
                valid.store(false);
            }
        }
    });
    for(int i = 0; i < count; i++) {
        input.t = i;
        input.q.setConstant(i);
        input.dq.setConstant(-i);
        input.torque_integral.setConstant(2.0 * i);
        while(!queue.try_push(input)) {
            std::this_thread::yield();
        }
    }
    consumer.join();
    require(valid.load(), "Concurrent queue exposed a torn or out-of-order state.");
}

void check_interval_torque()
{
    PandaRegressor dynamics;
    LSCLRCConfig config;
    config.Gamma *= 0.001;
    LSCLRCComputation computation(dynamics, config);
    LSCLRCInput input;
    input.q << 0.0, -0.5, 0.0, -1.8, 0.0, 1.5, 0.7;
    input.ref.q = input.q;
    input.last_tau_total.setConstant(0.5);
    computation.compute(input);
    input.t = 0.005;
    input.last_tau_total.setConstant(3.0);
    // 前 2 ms 力矩 1、后 3 ms 力矩 2：区间平均应为 1.6，不是末值 3。
    input.torque_integral.setConstant(0.002 * 1.0 + 0.003 * 2.0);
    const LSCLRCResult result = computation.compute(input);
    require(std::abs(result.dt - 0.005) < 1e-12 && result.tau_learning.isConstant(1.6, 1e-12),
        "Asynchronous learning did not use the time-weighted interval torque.");

    PandaLSCLRC controller(dynamics, config);
    controller.set_applied_torque(JointVector::Constant(0.5));
    controller.compute_control(0.0, input.q, input.dq, input.ref);
    controller.set_applied_torque(input.last_tau_total);
    const JointVector mean = JointVector::Constant(1.6);
    const JointVector tau = controller.compute_control(0.005, input.q, input.dq, input.ref, 0.005, &mean);
    const double gain = 1.0 - std::exp(-config.alpha * 0.005);
    require(controller.diagnostics().tau_f.isApprox(gain * mean, 1e-12),
        "Filter ignored the explicit interval input.");
    require(result.tau.isApprox(tau, 1e-12) && result.w.isApprox(controller.parameters(), 1e-12),
        "Calculation wrapper changed the controller equations.");

    // 跳过两个积压状态之后，仍应使用完整的 15 ms 区间。
    input.t = 0.020;
    input.torque_integral += 0.015 * JointVector::Constant(2.5);
    const LSCLRCResult skipped = computation.compute(input);
    require(std::abs(skipped.dt - 0.015) < 1e-12 && skipped.tau_learning.isConstant(2.5, 1e-12),
        "Skipping queued states broke the timestamp or cumulative torque alignment.");

    input.t = 0.027;
    input.torque_integral += 0.007 * JointVector::Constant(1.75);
    const LSCLRCResult gap = computation.compute(input);
    require(std::abs(gap.dt - 0.007) < 1e-12 && gap.tau_learning.isConstant(1.75, 1e-12)
        && gap.tau.allFinite() && gap.w.allFinite(),
        "A 7 ms callback gap broke calculation interval or learning input alignment.");
}

void check_worker()
{
    PandaRegressor dynamics;
    LSCLRCConfig config;
    LSCLRCWorker worker(dynamics, config);
    LSCLRCResult result;
    result.tau.setConstant(0.75);
    result.w = PandaRegressor::nominal_parameters();
    const ParameterVector initial_w = result.w;
    require(!worker.receive_latest(result) && result.tau.isConstant(0.75)
        && result.w == initial_w && result.t == 0.0,
        "Empty worker did not retain the startup holding torque and parameters.");
    worker.start(false); // 离线测试无需实时权限，也不连接机器人
    LSCLRCInput input;
    input.q << 0.0, -0.5, 0.0, -1.8, 0.0, 1.5, 0.7;
    input.ref.q = input.q;
    require(worker.submit(input), "Worker rejected its first state.");
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool received = false;
    while(std::chrono::steady_clock::now() < deadline && !worker.has_failed()) {
        if(worker.receive_latest(result)) {
            received = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    require(received && result.t == 0.0 && result.w.allFinite(), "Worker did not publish a complete result.");
    const LSCLRCResult held = result;
    // 模拟连续 20 个 1 kHz 发送周期没有新结果，超过原 4 ms 年龄上限。
    for(int tick = 1; tick <= 20; tick++) {
        fig3::check_control_timestamp(tick * config.dt, result.t);
        require(!worker.receive_latest(result) && result.t == held.t && result.dt == held.dt
            && result.tau == held.tau && result.w == held.w
            && result.tau_learning == held.tau_learning
            && result.calculation_us == held.calculation_us
            && result.history_fraction == held.history_fraction,
            "Missing calculation changed held torque, parameters or diagnostics.");
    }
    // 恢复后用整个保持区间推进学习，不能把 20 ms 当成名义 1 ms。
    input.t = 0.020;
    input.last_tau_total.setConstant(3.0);
    input.torque_integral.setConstant(0.020 * 1.5);
    require(worker.submit(input), "Worker rejected state after holding.");
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    received = false;
    while(std::chrono::steady_clock::now() < deadline && !worker.has_failed()) {
        if(worker.receive_latest(result)) {
            received = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    require(received && result.t == input.t && std::abs(result.dt - 0.020) < 1e-12
        && result.tau_learning.isConstant(1.5, 1e-12) && result.w.allFinite(),
        "Worker did not resume with the full elapsed interval and mean torque.");
    // 重复时间戳必须让工作线程报告错误，而不是终止进程或无限等待。
    require(worker.submit(input), "Worker rejected duplicate-state test input.");
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(std::chrono::steady_clock::now() < deadline && !worker.has_failed()) {
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    require(worker.has_failed() && !worker.failure_message().empty(), "Worker exception was not published.");
    worker.stop();
}

void check_hundred_second_1khz()
{
    PandaRegressor dynamics;
    LSCLRCConfig config;
    config.initial_W_hat = PandaRegressor::nominal_parameters();
    LSCLRCComputation computation(dynamics, config);
    JointVector center;
    center << 0.0, -0.5, 0.0, -1.8, 0.0, 1.5, 0.7;
    JointReference ref;
    ref.q = center;
    const JointVector amplitude = JointVector::Constant(0.08);
    LSCLRCInput input;
    double previous_control_t = -1.0;
    int computations = 0;
    const auto start = std::chrono::steady_clock::now();
    // 100001 次 1 kHz 状态采样与计算，轮间不重置。
    for(int ms = 0; ms <= 100000; ms++) {
        const double t = ms * config.dt;
        if(ms > 0) {
            fig3::advance_reference(ref, center, amplitude, t - config.dt, t, 10);
        }
        ref.ddq = 36.0 * (fig3::position_command(t, center, amplitude, 10) - ref.q) - 12.0 * ref.dq;
        input.last_tau_total = dynamics.inverse_regressor(ref.q, ref.dq, ref.ddq) * config.initial_W_hat;
        if(ms > 0) {
            input.torque_integral += config.dt * input.last_tau_total;
        }
        input.t = t;
        input.q = ref.q;
        input.dq = ref.dq;
        input.ref = ref;
        const LSCLRCResult result = computation.compute(input);
        require(std::abs(result.dt - config.dt) < 1e-10,
            "Nominal 1 kHz calculation used an incorrect interval.");
        require(result.w.allFinite() && result.w.norm() <= config.parameter_bound + 1e-10,
            "1 kHz parameters are non-finite or outside the projection bound.");
        if(t >= config.tau_d) {
            const double selected_sigma = result.using_frozen_history ? result.best_sigma : result.sigma_min;
            require(result.history_fraction > 1.0 - 1e-10
                && result.history_update_active == (selected_sigma >= config.history_activation_threshold),
                "1 kHz control lost its full history or bypassed the excitation threshold.");
        }
        previous_control_t = t;
        computations++;
    }
    require(computations == 100001 && previous_control_t == 100.0,
        "1 kHz calculation did not cover all ten rounds and 100 seconds.");
    std::cout << "Offline 100 s: 100001 sender samples, " << computations
              << " computations at 1000 Hz, elapsed="
              << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() << " s\n";
}

int main()
{
    try {
        check_sender_timing();
        check_queue();
        check_interval_torque();
        check_worker();
        check_hundred_second_1khz();
        std::cout << "Asynchronous queue, worker and interval alignment checks passed.\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
