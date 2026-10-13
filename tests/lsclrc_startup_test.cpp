#include "Franka_startup.h"

#include <array>
#include <iostream>
#include <sstream>
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

template <typename Predicate>
franka::Errors make_errors(Predicate predicate)
{
    // 从SDK公开字段查找测试标志，不依赖其枚举编号。
    std::array<bool, 41> flags{};
    for(std::size_t i = 0; i < flags.size(); i++) {
        std::array<bool, 41> single{};
        single[i] = true;
        flags[i] = predicate(franka::Errors(single));
    }
    return franka::Errors(flags);
}

franka::RobotState idle_state()
{
    franka::RobotState state;
    state.robot_mode = franka::RobotMode::kIdle;
    state.q = {0.0, -0.5, 0.0, -1.8, 0.0, 1.5, 0.7};
    return state;
}

struct RobotStub {
    franka::RobotState after_recovery = idle_state();
    int recoveries = 0;
    int reads = 0;
    bool fail_recovery = false;

    void automaticErrorRecovery()
    {
        recoveries++;
        if(fail_recovery) {
            throw std::runtime_error("Recovery connection lost.");
        }
    }

    franka::RobotState readOnce()
    {
        reads++;
        return after_recovery;
    }
};

void check_recovery()
{
    const auto communication = make_errors([](const franka::Errors& e) {
        return e.communication_constraints_violation;
    });
    require(static_cast<bool>(communication), "Communication fixture did not set a flag.");
    for(bool active : {true, false}) {
        auto state = idle_state();
        state.robot_mode = franka::RobotMode::kReflex;
        state.last_motion_errors = communication;
        if(active) {
            state.current_errors = communication;
        }
        RobotStub robot;
        robot.after_recovery.q[0] = 0.1;
        std::ostringstream log;
        hardware::ensure_ready(robot, state, log);
        require(robot.recoveries == 1 && robot.reads == 1
            && state.robot_mode == franka::RobotMode::kIdle && state.q[0] == 0.1,
            "Communication recovery did not refresh the initial state exactly once.");
        require(log.str().find("communication_constraints_violation") != std::string::npos,
            "Startup diagnostics lost the cause of Reflex.");
    }

    auto state = idle_state();
    RobotStub robot;
    std::ostringstream log;
    state.last_motion_errors = communication;
    hardware::ensure_ready(robot, state, log);
    require(robot.recoveries == 0 && robot.reads == 0,
        "Idle startup recovered a stale historical error.");
}

void check_recovery_gates()
{
    const auto communication = make_errors([](const franka::Errors& e) {
        return e.communication_constraints_violation;
    });
    const auto mixed = make_errors([](const franka::Errors& e) {
        return e.communication_constraints_violation || e.joint_reflex;
    });
    const auto torque = make_errors([](const franka::Errors& e) {
        return e.controller_torque_discontinuity;
    });
    for(auto mode : {franka::RobotMode::kUserStopped, franka::RobotMode::kMove,
        franka::RobotMode::kGuiding, franka::RobotMode::kOther,
        franka::RobotMode::kAutomaticErrorRecovery}) {
        auto state = idle_state();
        state.robot_mode = mode;
        state.current_errors = communication;
        RobotStub robot;
        std::ostringstream log;
        require_rejected([&] { hardware::ensure_ready(robot, state, log); },
            "Non-Reflex startup bypassed mode gating.");
        require(robot.recoveries == 0, "Recovery ran outside Reflex.");
    }
    for(const auto& errors : {mixed, torque, franka::Errors()}) {
        auto state = idle_state();
        state.robot_mode = franka::RobotMode::kReflex;
        state.current_errors = errors;
        RobotStub robot;
        std::ostringstream log;
        require_rejected([&] { hardware::ensure_ready(robot, state, log); },
            "Non-communication Reflex bypassed startup checks.");
        require(robot.recoveries == 0, "Recovery cleared mixed, torque or unknown errors.");
    }
    auto state = idle_state();
    state.current_errors = communication;
    RobotStub robot;
    std::ostringstream log;
    require_rejected([&] { hardware::ensure_ready(robot, state, log); },
        "Idle with active errors was accepted.");
    state.robot_mode = franka::RobotMode::kReflex;
    state.dq[0] = 0.1;
    require_rejected([&] { hardware::ensure_ready(robot, state, log); },
        "Recovery accepted a moving robot.");
    require(robot.recoveries == 0, "Moving robot reached recovery API.");
    state.dq[0] = 0.0;
    state.q[3] = -0.05;
    require_rejected([&] { hardware::ensure_ready(robot, state, log); },
        "Recovery accepted a robot outside the position margin.");
    require(robot.recoveries == 0, "Position violation reached recovery API.");
}

void check_recovery_failure()
{
    auto state = idle_state();
    state.robot_mode = franka::RobotMode::kReflex;
    state.current_errors = make_errors([](const franka::Errors& e) {
        return e.communication_constraints_violation;
    });
    RobotStub robot;
    robot.after_recovery = state;
    std::ostringstream log;
    require_rejected([&] { hardware::ensure_ready(robot, state, log); },
        "Persistent Reflex was accepted after recovery.");
    require(robot.recoveries == 1 && robot.reads == 1, "Persistent Reflex retried recovery.");
    robot = RobotStub();
    robot.fail_recovery = true;
    require_rejected([&] { hardware::ensure_ready(robot, state, log); },
        "Recovery exception was swallowed.");
    require(robot.recoveries == 1 && robot.reads == 0, "Recovery connection failure was retried.");
}

int main()
{
    try {
        check_recovery();
        check_recovery_gates();
        check_recovery_failure();
        LSCLRCConfig config;
        JointVector limits;
        limits << 87, 87, 87, 87, 12, 12, 12;
        require(config.torque_limit == limits, "Default Panda torque limits changed.");
        std::cout << "Communication recovery, startup gates and diagnostics checks passed.\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
