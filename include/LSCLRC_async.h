#ifndef GUO_LSCLRC_ASYNC_H
#define GUO_LSCLRC_ASYNC_H

#include "LSCLRC_control.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <pthread.h>
#include <string>
#include <stdexcept>
#include <thread>

namespace clrc_guo {

// 单生产者、单消费者；满队列时返回 false，发送线程不等待、不覆盖未读槽。
template <typename T, unsigned Capacity = 4>
class SpscQueue {
    static_assert(Capacity >= 2, "SPSC queue needs at least two slots.");
    static_assert(std::atomic<unsigned>::is_always_lock_free, "Indices must be lock-free.");
    std::array<T, Capacity> slots;
    alignas(64) std::atomic<unsigned> write_index{0};
    alignas(64) std::atomic<unsigned> read_index{0};

public:
    bool try_push(const T& value)
    {
        const unsigned write = write_index.load(std::memory_order_relaxed);
        const unsigned next = (write + 1) % Capacity;
        if(next == read_index.load(std::memory_order_acquire)) {
            return false;
        }
        slots[write] = value;
        write_index.store(next, std::memory_order_release);
        return true;
    }

    bool try_pop(T& value)
    {
        const unsigned read = read_index.load(std::memory_order_relaxed);
        if(read == write_index.load(std::memory_order_acquire)) {
            return false;
        }
        value = slots[read];
        read_index.store((read + 1) % Capacity, std::memory_order_release);
        return true;
    }
};

struct LSCLRCInput {
    double t = 0.0;
    JointVector q = JointVector::Zero();
    JointVector dq = JointVector::Zero();
    JointReference ref;
    JointVector last_tau_total = JointVector::Zero();
    // 发送线程逐周期积分 tau_J_d + gravity；时间戳对应刚结束的区间。
    JointVector torque_integral = JointVector::Zero();
};

// 只传发送/记录需要的结果，不跨线程复制完整 Yf 和历史矩阵。
struct LSCLRCResult {
    double t = 0.0;
    double dt = 0.0;
    double calculation_us = 0.0;
    double max_calculation_us = 0.0;
    JointVector tau = JointVector::Zero();
    JointVector tau_learning = JointVector::Zero();
    ParameterVector w = ParameterVector::Zero();
    double sigma_min = 0.0;
    double best_sigma = 0.0;
    bool ie_detected = false;
    bool history_update_active = false;
    int information_rank = 0;
    double parameter_update_norm = 0.0;
    double prediction_error_norm = 0.0;
    double history_fraction = 0.0;
    double direct_learning_term_norm = 0.0;
    double history_learning_term_norm = 0.0;
    int pinv_rank = 0;
    bool using_frozen_history = false;
    double least_squares_term_norm = 0.0;
};

// 仅计算线程访问；独立于调度逻辑，便于离线核对采样与力矩对齐。
class LSCLRCComputation {
    PandaLSCLRC controller;
    double previous_t = -1.0;
    JointVector previous_integral = JointVector::Zero();
    double max_us = 0.0;

public:
    LSCLRCComputation(const PandaRegressor& dynamics, const LSCLRCConfig& config)
        : controller(dynamics, config) {}

    LSCLRCResult compute(const LSCLRCInput& input)
    {
        const auto start = std::chrono::steady_clock::now();
        LSCLRCResult result;
        result.t = input.t;
        result.dt = previous_t < 0.0 ? controller.timestep() : input.t - previous_t;
        if(!std::isfinite(result.dt) || result.dt <= 0.0 || !input.torque_integral.allFinite()) {
            throw std::invalid_argument("Invalid asynchronous control interval or torque integral.");
        }
        result.tau_learning = previous_t < 0.0 ? input.last_tau_total
            : JointVector((input.torque_integral - previous_integral) / result.dt);
        // 限速以最新反馈力矩为基准；滤波输入独立采用整个采样区间的平均力矩。
        controller.set_applied_torque(input.last_tau_total);
        const ParameterVector previous_w = controller.parameters();
        result.tau = controller.compute_control(input.t, input.q, input.dq, input.ref,
            result.dt, &result.tau_learning);
        const auto& d = controller.diagnostics();
        result.w = controller.parameters();
        result.sigma_min = d.sigma_min;
        result.best_sigma = d.best_sigma;
        result.ie_detected = d.ie_detected;
        result.history_update_active = d.history_update_active;
        result.information_rank = d.information_rank;
        result.parameter_update_norm = (result.w - previous_w).norm() / result.dt;
        result.prediction_error_norm = d.prediction_error.norm();
        result.history_fraction = d.history_fraction;
        result.direct_learning_term_norm = d.direct_learning_term_norm;
        result.history_learning_term_norm = d.history_learning_term_norm;
        result.pinv_rank = d.pinv_rank;
        result.using_frozen_history = d.using_frozen_history;
        result.least_squares_term_norm = d.least_squares_term.norm();
        previous_t = input.t;
        previous_integral = input.torque_integral;
        result.calculation_us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - start).count();
        max_us = std::max(max_us, result.calculation_us);
        result.max_calculation_us = max_us;
        return result;
    }
};

class LSCLRCWorker {
    static_assert(std::atomic<bool>::is_always_lock_free, "Worker flags must be lock-free.");
    LSCLRCComputation computation;
    SpscQueue<LSCLRCInput> inputs;
    SpscQueue<LSCLRCResult> outputs;
    std::atomic<bool> stopping{false};
    std::atomic<bool> failed{false};
    std::thread thread;
    std::string error;

public:
    LSCLRCWorker(const PandaRegressor& dynamics, const LSCLRCConfig& config)
        : computation(dynamics, config) {}
    ~LSCLRCWorker() { stop(); }

    void start(bool realtime = true)
    {
        if(thread.joinable() || stopping.load(std::memory_order_acquire)) {
            throw std::logic_error("Calculation worker can only be started once.");
        }
        thread = std::thread([this] {
            try {
                LSCLRCInput input;
                LSCLRCInput newer;
                while(!stopping.load(std::memory_order_acquire)) {
                    if(!inputs.try_pop(input)) {
                        // 只有计算线程等待，libfranka 发送回调不等待。
                        std::this_thread::sleep_for(std::chrono::microseconds(50));
                        continue;
                    }
                    while(inputs.try_pop(newer)) {
                        input = newer; // 跳过积压状态，使用真实时间差和累计力矩计算
                    }
                    if(!outputs.try_push(computation.compute(input))) {
                        throw std::runtime_error("Calculation output queue is full; sender is not consuming results.");
                    }
                }
            } catch(const std::exception& e) {
                error = e.what();
                failed.store(true, std::memory_order_release);
            }
        });
        if(realtime) {
            // libfranka 将发送线程设为更高的 FIFO 优先级；计算线程可被发送抢占。
            sched_param scheduling{};
            scheduling.sched_priority = 10;
            const int status = pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &scheduling);
            if(status != 0) {
                stop();
                throw std::runtime_error(std::string("Cannot set calculation thread realtime priority: ")
                    + std::strerror(status));
            }
        }
    }

    void stop()
    {
        stopping.store(true, std::memory_order_release);
        if(thread.joinable()) {
            thread.join();
        }
    }

    bool submit(const LSCLRCInput& input) { return inputs.try_push(input); }
    bool receive(LSCLRCResult& result) { return outputs.try_pop(result); }
    // 非阻塞读取最新完成结果；队列为空时保留调用者上次的完整结果。
    bool receive_latest(LSCLRCResult& result)
    {
        LSCLRCResult available;
        bool updated = false;
        while(outputs.try_pop(available)) {
            result = available;
            updated = true;
        }
        return updated;
    }
    bool has_failed() const { return failed.load(std::memory_order_acquire); }
    const std::string& failure_message() const { return error; } // 仅在 has_failed() 后读取
};

} // namespace clrc_guo

#endif
