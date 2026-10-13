#include "LSCLRC_control.h"
#include "Fig3_experiment.h"
#include "Franka_startup.h"
#ifdef LSCLRC_ASYNC_COMPUTE
#include "LSCLRC_async.h"
#endif

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace clrc_guo;

// 沿用参考项目 Fig. 3(c) 的 Franka 实验：启动姿态为中心，1~2 s 负脉冲，5~6 s 正脉冲。
// 单位为 rad；各轴幅值可独立修改，设为 0 表示保持该轴的启动位置。
constexpr std::array<double, 7> AMPLITUDE = {0.15, 0.20, 0.10, 0.10, 0.08, 0.08, 0.08};
constexpr double DT = 0.001;
#ifndef LSCLRC_EXPERIMENT_ROUNDS
#define LSCLRC_EXPERIMENT_ROUNDS 1
#endif
// lsclrc_fig3 执行一轮；lsclrc_repeat 执行十轮，两者复用相同实机回调。
constexpr int ROUNDS = LSCLRC_EXPERIMENT_ROUNDS;
static_assert(ROUNDS > 0, "Experiment must contain at least one round.");
constexpr double RUN_TIME = ROUNDS * fig3::CYCLE_TIME;
#ifdef LSCLRC_ASYNC_COMPUTE
constexpr uint64_t MAX_PERIOD_MS = fig3::ASYNC_MAX_PERIOD_MS;
#else
constexpr uint64_t MAX_PERIOD_MS = fig3::SYNC_MAX_PERIOD_MS;
#endif

struct Sample {
    double t = 0.0;
    int round = 1;
    double round_time = 0.0;
    double period_ms = 0.0;
    double callback_us = 0.0;
    double command_success_rate = 0.0;
    JointVector qc = JointVector::Zero();
    JointVector q = JointVector::Zero();
    JointVector dq = JointVector::Zero();
    JointReference ref;
    JointVector tau_cmd = JointVector::Zero();
    JointVector tau_total = JointVector::Zero();
    JointVector tau_measured = JointVector::Zero();
    ParameterVector w = ParameterVector::Zero();
    double sigma_min = 0.0;
    double best_sigma = 0.0;
    bool ie_detected = false;
    bool history_update_active = false;
    int information_rank = 0;
    double parameter_update_norm = 0.0;  // 实际离散更新 / dt，包含投影与球裁剪
    double prediction_error_norm = 0.0;
    double history_fraction = 0.0;
    double direct_learning_term_norm = 0.0;
    double history_learning_term_norm = 0.0;
    int pinv_rank = 0;
    bool using_frozen_history = false;
    double least_squares_term_norm = 0.0;
#ifdef LSCLRC_ASYNC_COMPUTE
    double control_t = 0.0;
    double control_dt_ms = 0.0;
    double control_age_ms = 0.0;
    double control_us = 0.0;
    bool control_valid = false;
    bool control_update = false;
    JointVector tau_learning = JointVector::Zero();
#endif
};

// 控制结束后才写文件；回调中只填充提前分配的内存。
void save_csv(const std::string& path, const std::vector<Sample>& samples, std::size_t count)
{
    std::ofstream file(path);
    if(!file) {
        throw std::runtime_error("Cannot open CSV: " + path);
    }
    file << std::setprecision(12) << "t";
    for(const char* name : {"qc", "q", "dq", "qd", "dqd", "ddqd", "tau_cmd", "tau_total", "tau_measured"}) {
        for(int j = 1; j <= PANDA_DOF; j++) {
            file << ',' << name << j;
        }
    }
    for(int j = 1; j <= PANDA_PARAMETERS; j++) {
        file << ",W_hat" << j;
    }
    file << ",sigma_min,ie_detected,information_rank,period_ms,callback_us,command_success_rate"
         << ",best_sigma,history_update_active,round,round_time,parameter_update_norm"
         << ",prediction_error_norm,history_fraction,direct_learning_term_norm,history_learning_term_norm"
         << ",pinv_rank,using_frozen_history,least_squares_term_norm";
#ifdef LSCLRC_ASYNC_COMPUTE
    file << ",control_t,control_dt_ms,control_age_ms,control_us,control_valid,control_update";
    for(int j = 1; j <= PANDA_DOF; j++) {
        file << ",tau_learning" << j;
    }
#endif
    file << '\n';
    for(std::size_t k = 0; k < count; k++) {
        const Sample& s = samples[k];
        file << s.t;
        for(const JointVector* values : {&s.qc, &s.q, &s.dq, &s.ref.q, &s.ref.dq, &s.ref.ddq,
                                        &s.tau_cmd, &s.tau_total, &s.tau_measured}) {
            for(int j = 0; j < PANDA_DOF; j++) {
                file << ',' << (*values)(j);
            }
        }
        for(int j = 0; j < PANDA_PARAMETERS; j++) {
            file << ',' << s.w(j);
        }
        file << ',' << s.sigma_min << ',' << s.ie_detected << ',' << s.information_rank
             << ',' << s.period_ms << ',' << s.callback_us << ',' << s.command_success_rate
             << ',' << s.best_sigma << ',' << s.history_update_active
             << ',' << s.round << ',' << s.round_time << ',' << s.parameter_update_norm
             << ',' << s.prediction_error_norm << ',' << s.history_fraction
             << ',' << s.direct_learning_term_norm << ',' << s.history_learning_term_norm
             << ',' << s.pinv_rank << ',' << s.using_frozen_history << ',' << s.least_squares_term_norm;
#ifdef LSCLRC_ASYNC_COMPUTE
        file << ',' << s.control_t << ',' << s.control_dt_ms << ',' << s.control_age_ms
             << ',' << s.control_us << ',' << s.control_valid << ',' << s.control_update;
        for(int j = 0; j < PANDA_DOF; j++) {
            file << ',' << s.tau_learning(j);
        }
#endif
        file << '\n';
    }
    file.close();
    if(!file) {
        throw std::runtime_error("Cannot finish writing CSV: " + path);
    }
}

int main(int argc, char** argv)
{
    if(argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: " << argv[0] << " [robot_ip=172.16.0.2] [csv_path]"
#ifdef LSCLRC_ASYNC_COMPUTE
                  << " [control_hz=1000]"
#endif
                  << '\n' << ROUNDS << " round(s), " << RUN_TIME << " s total.\n";
        std::cout << "Read state only: " << argv[0] << " --status [robot_ip=172.16.0.2]\n"
                  << "Startup automatically recovers communication-only Reflex once.\n";
        return 0;
    }
    if(argc >= 2 && std::string(argv[1]) == "--status") {
        if(argc > 3) {
            std::cerr << "Usage: " << argv[0] << " --status [robot_ip]\n";
            return 1;
        }
        try {
            franka::Robot robot(argc == 3 ? argv[2] : "172.16.0.2");
            hardware::print_state(std::cout, robot.readOnce());
            return 0;
        } catch(const std::exception& e) {
            std::cerr << "Status read failed: " << e.what() << '\n';
            return 1;
        }
    }
    const std::string robot_ip = argc > 1 ? argv[1] : "172.16.0.2";
    const std::string csv_path = argc > 2 ? argv[2] : (ROUNDS == 1 ? "lsclrc_fig3.csv" : "lsclrc_repeat.csv");
    const std::size_t last_step = static_cast<std::size_t>(std::lround(RUN_TIME / DT));
    std::vector<Sample> samples(last_step + 1);
    std::size_t count = 0;
    int result = 0;
    uint64_t elapsed_ms = 0;
    uint64_t last_period_ms = 0;
    uint64_t max_period_ms = 0;
    std::size_t gap_count = 0;
    double max_callback_us = 0.0;
    double success_rate = 0.0;
    JointVector last_q = JointVector::Zero();
    bool have_state = false;
    franka::RobotState last_state;
    std::unique_ptr<franka::Robot> robot;
    std::ostringstream failure_report;
    const auto record_failure = [&](const std::exception& e, const char* kind) {
        result = 1;
        failure_report << kind << ": " << e.what() << '\n';
        if(have_state) {
            failure_report << "Last received state: ";
            hardware::print_state(failure_report, last_state);
        }
        std::cerr << kind << ": " << e.what() << '\n';
    };
#ifdef LSCLRC_ASYNC_COMPUTE
    double control_hz = 1000.0;
    double max_calculation_us = 0.0;
    std::size_t input_drops = 0;
#endif

    try {
#ifndef LSCLRC_ASYNC_COMPUTE
        if(argc > 3) {
            throw std::invalid_argument("Usage: lsclrc_fig3 [robot_ip] [csv_path]");
        }
#endif
#ifdef LSCLRC_ASYNC_COMPUTE
        if(argc > 4) {
            throw std::invalid_argument("Usage: lsclrc_repeat [robot_ip] [csv_path] [control_hz=1000]");
        }
        if(argc > 3) {
            std::size_t parsed = 0;
            control_hz = std::stod(argv[3], &parsed);
            if(parsed != std::string(argv[3]).size()) {
                throw std::invalid_argument("Invalid calculation frequency.");
            }
        }
        if(!std::isfinite(control_hz) || control_hz < 100.0 || control_hz > 1000.0) {
            throw std::invalid_argument("Calculation frequency must be in [100, 1000] Hz.");
        }
        const double control_period = 1.0 / control_hz;
#endif
        // 沿用 tau_zero_input.cpp：直接建立连接，保留机器人已有碰撞保护设置。
        robot = std::make_unique<franka::Robot>(robot_ip);
        last_state = robot->readOnce();
        have_state = true;
        last_q = Eigen::Map<const JointVector>(last_state.q.data());
        success_rate = last_state.control_command_success_rate;
        hardware::ensure_ready(*robot, last_state, std::cout);
        const auto& initial_state = last_state;
        last_q = Eigen::Map<const JointVector>(initial_state.q.data());
        success_rate = initial_state.control_command_success_rate;
        const JointVector amplitude = Eigen::Map<const JointVector>(AMPLITUDE.data());
        fig3::check_position(Eigen::Map<const JointVector>(initial_state.q.data()), amplitude);
        fig3::check_stationary(Eigen::Map<const JointVector>(initial_state.dq.data()));
        franka::Model model = robot->loadModel();
        PandaRegressor dynamics;
        LSCLRCConfig config;                     // 沿用已有 Franka tuned 增益
        config.dt = DT;
        // 默认87/12 N m限幅；实机入口另开启变化率限制。
        config.torque_rate_limit.setConstant(800.0);
        // 实机不从零总力矩开始：零估计随后减去 g 会抵消内置重力补偿。
        config.initial_W_hat = PandaRegressor::nominal_parameters();
#ifdef LSCLRC_ASYNC_COMPUTE
        // 预检查在实时回调外执行；第一帧再次检查静止及启动姿态未改变。
        const JointVector initial_q = Eigen::Map<const JointVector>(initial_state.q.data());
        const JointVector nominal_hold = dynamics.inverse_regressor(
            initial_q, JointVector::Zero(), JointVector::Zero()) * config.initial_W_hat;
        const auto initial_gravity_array = model.gravity(initial_state);
        const JointVector initial_gravity = Eigen::Map<const JointVector>(initial_gravity_array.data());
        if((nominal_hold - initial_gravity).cwiseAbs().maxCoeff() > 3.0) {
            throw std::runtime_error("Nominal holding torque differs from Franka gravity by more than 3 Nm. "
                "Check parameter/model pairing, payload and base orientation before starting.");
        }
        LSCLRCWorker worker(dynamics, config);
        LSCLRCResult latest;
        latest.tau = nominal_hold;
        latest.w = config.initial_W_hat;
        bool control_valid = false;
        JointVector torque_integral = JointVector::Zero();
        double next_control_t = 0.0;
        worker.start();
#else
        PandaLSCLRC controller(dynamics, config);
#endif
        JointVector center = JointVector::Zero();
        JointReference reference;
        std::cout << "Guo LS-CLRC experiment: " << ROUNDS << " round(s) x " << fig3::CYCLE_TIME
                  << " s = " << RUN_TIME << " s. Learning state is retained between rounds.\n";
        std::cout << "History activation minimum eigenvalue >= " << config.history_activation_threshold
                  << "; IE storage threshold = " << config.ie_threshold << ".\n";
#ifdef LSCLRC_ASYNC_COMPUTE
        std::cout << "Threads: Franka send=1000 Hz, LS-CLRC calculation=" << control_hz
                  << " Hz; callback-gap stop=>" << MAX_PERIOD_MS
                  << " ms; hold previous result until a new calculation completes.\n";
#endif

        robot->control(
            [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
                const auto callback_start = std::chrono::steady_clock::now();
                const JointVector q = Eigen::Map<const JointVector>(state.q.data());
                const JointVector dq = Eigen::Map<const JointVector>(state.dq.data());
                have_state = true;
                last_state = state; // 只复制，不在实时回调格式化错误或写文件
                last_q = q;
                success_rate = state.control_command_success_rate;
                last_period_ms = period.toMSec();
                max_period_ms = std::max(max_period_ms, last_period_ms);
                if(count > 0 && last_period_ms != 1) {
                    gap_count++;
                }
                fig3::check_callback_period(last_period_ms, count == 0, MAX_PERIOD_MS,
                    (elapsed_ms + last_period_ms) * DT);
                fig3::check_position(q);
                const auto gravity_array = model.gravity(state);
                const JointVector gravity = Eigen::Map<const JointVector>(gravity_array.data());
#ifndef LSCLRC_ASYNC_COMPUTE
                const double sample_dt = count == 0 ? DT : period.toSec();
#endif
                const double previous_t = static_cast<double>(elapsed_ms) * DT;

                if(count == 0) {
                    fig3::check_position(q, amplitude);
                    fig3::check_stationary(dq);
                    center = q;
                    reference.q = q;           // 从测量姿态开始，初始参考速度为零
#ifdef LSCLRC_ASYNC_COMPUTE
                    if((q - initial_q).cwiseAbs().maxCoeff() > 0.01) {
                        throw std::runtime_error("Startup pose changed after preflight; restart from a stationary pose.");
                    }
#else
                    const JointVector nominal_hold = dynamics.inverse_regressor(
                        q, JointVector::Zero(), JointVector::Zero()) * config.initial_W_hat;
                    const double holding_error = (nominal_hold - gravity).cwiseAbs().maxCoeff();
                    if(holding_error > 3.0) {
                        throw std::runtime_error("Nominal holding torque differs from Franka gravity by "
                            + std::to_string(holding_error) + " Nm (limit 3 Nm). "
                            "Check parameter/model pairing, payload and base orientation before starting.");
                    }
                    controller.set_applied_torque(
                        Eigen::Map<const JointVector>(state.tau_J_d.data()) + gravity);
#endif
                } else {
                    elapsed_ms += last_period_ms;
                    fig3::advance_reference(reference, center, amplitude, previous_t, elapsed_ms * DT, ROUNDS);
                }
                const double t = static_cast<double>(elapsed_ms) * DT;
                const JointVector qc = fig3::position_command(t, center, amplitude, ROUNDS);
                reference.ddq = 36.0 * (qc - reference.q) - 12.0 * reference.dq;
                Eigen::Index error_joint = 0;
                const double max_error = (q - reference.q).cwiseAbs().maxCoeff(&error_joint);
                if(max_error > 0.25) {
                    throw std::runtime_error("Tracking error exceeds 0.25 rad on joint "
                        + std::to_string(error_joint + 1) + ": error="
                        + std::to_string(reference.q(error_joint) - q(error_joint))
                        + " rad, dq=" + std::to_string(dq(error_joint))
                        + " rad/s, t=" + std::to_string(t) + " s; stopping experiment.");
                }

#ifdef LSCLRC_ASYNC_COMPUTE
                // tau_J_d 是当前状态中已生效的接口力矩，用于估算刚结束区间的输入。
                const JointVector last_tau_total = Eigen::Map<const JointVector>(state.tau_J_d.data()) + gravity;
                if(count > 0) {
                    torque_integral += period.toSec() * last_tau_total;
                }
                if(worker.has_failed()) {
                    throw std::runtime_error("Calculation thread: " + worker.failure_message());
                }
                const bool control_update = worker.receive_latest(latest);
                if(control_update) {
                    control_valid = true;
                    max_calculation_us = latest.max_calculation_us;
                }
                // 未完成新计算时保留 latest，不因结果年龄停止，也不重复更新参数。
                fig3::check_control_timestamp(t, latest.t);
                if(t + 1e-10 >= next_control_t) {
                    LSCLRCInput input;
                    input.t = t;
                    input.q = q;
                    input.dq = dq;
                    input.ref = reference;
                    input.last_tau_total = last_tau_total;
                    input.torque_integral = torque_integral;
                    if(!worker.submit(input)) {
                        input_drops++;
                    }
                    // 按机器人时间调度，漏包后不补算虚构的多帧状态。
                    next_control_t = (std::floor((t + 1e-10) / control_period) + 1.0) * control_period;
                }
                const JointVector tau = latest.tau;
#else
                const ParameterVector previous_w = controller.parameters();
                const JointVector tau = controller.compute_control(t, q, dq, reference, sample_dt);
#endif
                std::array<double, 7> command{};
                JointVector applied;
                for(int j = 0; j < PANDA_DOF; j++) {
                    // LS-CLRC 输出总力矩；保留 Franka 内置重力补偿，只减一次当前 g_Franka。
                    // tau_cmd = tau_total - g_Franka；辨识输入加回同一重力。
                    // 同时约束接口力矩、含重力总力矩，以及相对上一指令的变化率。
                    const double limit = config.torque_limit(j);
                    // FCI 每 1 ms 检查新指令与 tau_J_d 的跳变量；漏包也不放宽。
                    const double delta = config.torque_rate_limit(j) * DT;
                    const double low = std::max({-limit, -limit - gravity(j), state.tau_J_d[j] - delta});
                    const double high = std::min({limit, limit - gravity(j), state.tau_J_d[j] + delta});
                    if(low > high) {
                        throw std::runtime_error("Torque bounds cannot be satisfied.");
                    }
                    command[j] = std::clamp(tau(j) - gravity(j), low, high);
                    applied(j) = command[j] + gravity(j);
                }
#ifndef LSCLRC_ASYNC_COMPUTE
                controller.set_applied_torque(applied);  // 学习使用最终指令的总力矩定义
#endif

                if(count >= samples.size()) {
                    throw std::runtime_error("Experiment sample buffer is full.");
                }
                Sample& s = samples[count];
                s.t = t;
                s.round = fig3::cycle_index(t, ROUNDS);
                s.round_time = fig3::cycle_time(t, ROUNDS);
                s.qc = qc;
                s.q = q;
                s.dq = dq;
                s.ref = reference;
                s.tau_cmd = Eigen::Map<const JointVector>(command.data());
                s.tau_total = applied;
                s.tau_measured = Eigen::Map<const JointVector>(state.tau_J.data());
#ifdef LSCLRC_ASYNC_COMPUTE
                s.w = latest.w;
                s.sigma_min = latest.sigma_min;
                s.best_sigma = latest.best_sigma;
                s.ie_detected = latest.ie_detected;
                s.history_update_active = latest.history_update_active;
                s.information_rank = latest.information_rank;
                s.parameter_update_norm = latest.parameter_update_norm;
                s.prediction_error_norm = latest.prediction_error_norm;
                s.history_fraction = latest.history_fraction;
                s.direct_learning_term_norm = latest.direct_learning_term_norm;
                s.history_learning_term_norm = latest.history_learning_term_norm;
                s.pinv_rank = latest.pinv_rank;
                s.using_frozen_history = latest.using_frozen_history;
                s.least_squares_term_norm = latest.least_squares_term_norm;
                s.control_t = latest.t;
                s.control_dt_ms = latest.dt * 1000.0;
                s.control_age_ms = (t - latest.t) * 1000.0;
                s.control_us = latest.calculation_us;
                s.control_valid = control_valid;
                s.control_update = control_update;
                s.tau_learning = latest.tau_learning;
#else
                s.w = controller.parameters();
                s.sigma_min = controller.diagnostics().sigma_min;
                s.best_sigma = controller.diagnostics().best_sigma;
                s.ie_detected = controller.diagnostics().ie_detected;
                s.history_update_active = controller.diagnostics().history_update_active;
                s.information_rank = controller.diagnostics().information_rank;
                s.parameter_update_norm = (s.w - previous_w).norm() / sample_dt;
                s.prediction_error_norm = controller.diagnostics().prediction_error.norm();
                s.history_fraction = controller.diagnostics().history_fraction;
                s.direct_learning_term_norm = controller.diagnostics().direct_learning_term_norm;
                s.history_learning_term_norm = controller.diagnostics().history_learning_term_norm;
                s.pinv_rank = controller.diagnostics().pinv_rank;
                s.using_frozen_history = controller.diagnostics().using_frozen_history;
                s.least_squares_term_norm = controller.diagnostics().least_squares_term.norm();
#endif
                s.period_ms = last_period_ms;
                s.command_success_rate = success_rate;
                s.callback_us = std::chrono::duration<double, std::micro>(
                    std::chrono::steady_clock::now() - callback_start).count();
                max_callback_us = std::max(max_callback_us, s.callback_us);
                count++;

                const franka::Torques output(command);
                return elapsed_ms >= last_step ? franka::MotionFinished(output) : output;
            },
            // false 仅关闭 SDK 额外限速；内置重力补偿继续生效。
            false, franka::kMaxCutoffFrequency  // 已自行限速，关闭额外限速及低通，避免学习输入不一致
        );
#ifdef LSCLRC_ASYNC_COMPUTE
        worker.stop();
        if(worker.has_failed()) {
            throw std::runtime_error("Calculation thread: " + worker.failure_message());
        }
#endif
    }
    catch(const franka::ControlException& e) {
        // SDK日志可能包含回调未收到的最终Reflex状态。
        if(!e.log.empty()) {
            last_state = e.log.back().state;
            have_state = true;
        }
        record_failure(e, "Franka control error");
    }
    catch(const franka::NetworkException& e) {
        record_failure(e, "Franka network error");
        // 关闭坏连接；控制线程已退出，仅重新读取错误状态，不续跑或恢复。
        robot.reset();
        if(have_state) {
            try {
                franka::Robot diagnostic_robot(robot_ip);
                const auto stopped_state = diagnostic_robot.readOnce();
                failure_report << "After read-only reconnect: ";
                hardware::print_state(failure_report, stopped_state);
                std::cerr << "After read-only reconnect: ";
                hardware::print_state(std::cerr, stopped_state);
            } catch(const std::exception& status_error) {
                failure_report << "Read-only reconnect failed: " << status_error.what() << '\n';
                std::cerr << "Read-only reconnect failed: " << status_error.what() << '\n';
            }
        }
        std::cerr << "TCP/UDP transport failed; check the FCI connection, link and realtime/network jitter. "
                  << "Torque-limit changes cannot repair a broken connection.\n";
    }
    catch(const std::exception& e) {
        record_failure(e, "Control error");
    }

    if(have_state) {
        std::cerr << "Last received state: ";
        hardware::print_state(std::cerr, last_state);
        std::cerr << "Timing: last period=" << last_period_ms << " ms, max period=" << max_period_ms
                  << " ms, gaps=" << gap_count << ", max callback=" << max_callback_us
                  << " us, command success rate=" << success_rate << "\nLast q: " << last_q.transpose() << '\n';
    }
    const double recorded_time = count == 0 ? 0.0 : samples[count - 1].t;
#ifdef LSCLRC_ASYNC_COMPUTE
    std::cout << "Calculation thread: configured=" << control_hz
              << " Hz, max compute=" << max_calculation_us << " us, input queue drops=" << input_drops << '\n';
#endif
    const bool completed = result == 0 && elapsed_ms >= last_step;
    std::cout << (completed ? "Experiment completed: " : "Experiment stopped early: ")
              << "last recorded robot time=" << recorded_time << " / " << RUN_TIME
              << " s, " << count << " samples.\n";
    if(count == 0) {
        std::cerr << "No control samples; existing CSV left untouched: " << csv_path << '\n';
        return result;
    }
    // 控制异常时也保存已完成的样本，便于检查停止前的状态。
    try {
        save_csv(csv_path, samples, count);
        std::cout << "Saved " << count << " samples to " << csv_path << '\n';
        if(result != 0) {
            const std::string report_path = csv_path + ".status.txt";
            std::ofstream report(report_path);
            report << failure_report.str()
                   << "Recorded time=" << recorded_time << " s, samples=" << count
                   << ", max period=" << max_period_ms << " ms, gaps=" << gap_count
                   << ", max callback=" << max_callback_us << " us\n";
            report.close();
            if(!report) {
                throw std::runtime_error("Cannot write failure report: " + report_path);
            }
            std::cout << "Saved failure report to " << report_path << '\n';
        }
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        result = 1;
    }
    return result;
}
