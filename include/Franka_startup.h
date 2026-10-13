#ifndef GUO_FRANKA_STARTUP_H
#define GUO_FRANKA_STARTUP_H

#include "Fig3_experiment.h"
#include <franka/robot_state.h>

#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace clrc_guo::hardware {

inline void print_state(std::ostream& output, const franka::RobotState& state)
{
    output << "robot_mode=" << state.robot_mode
           << ", current_errors=" << state.current_errors
           << ", last_motion_errors=" << state.last_motion_errors
           << ", command_success_rate=" << state.control_command_success_rate
           << ", robot_time=" << state.time.toSec() << " s\n";
}

inline bool only_communication_error(const franka::Errors& errors)
{
    // Errors不公开底层数组；SDK的JSON转换包含所有标志，避免遗漏其他错误。
    return errors.communication_constraints_violation
        && static_cast<std::string>(errors) == "[\"communication_constraints_violation\"]";
}

inline bool can_recover_communication(const franka::RobotState& state)
{
    if(state.robot_mode != franka::RobotMode::kReflex) {
        return false;
    }
    if(static_cast<bool>(state.current_errors)) {
        return only_communication_error(state.current_errors);
    }
    return only_communication_error(state.last_motion_errors);
}

inline std::string startup_error(const franka::RobotState& state)
{
    std::ostringstream message;
    message << "Robot is not ready for torque control: ";
    print_state(message, state);
    if(state.robot_mode == franka::RobotMode::kUserStopped) {
        message << "Release the user stop and enable the robot/FCI in Desk before restarting.";
    } else if(state.robot_mode == franka::RobotMode::kMove
        || state.robot_mode == franka::RobotMode::kGuiding) {
        message << "End the existing motion/guiding session before restarting.";
    } else {
        message << "Resolve the listed errors in Desk and check FCI activation before restarting.";
    }
    return message.str();
}

// 只在启动新实验之前执行；断连的旧实验不恢复或接着发送力矩。
// Robot可替换为测试对象，离线验证状态门控与恢复后的重新检查。
template <typename Robot>
void ensure_ready(Robot& robot, franka::RobotState& state, std::ostream& output)
{
    print_state(output, state);
    if(can_recover_communication(state)) {
        fig3::check_stationary(Eigen::Map<const JointVector>(state.dq.data()));
        fig3::check_position(Eigen::Map<const JointVector>(state.q.data()));
        output << "Startup: recovering communication-only Reflex once.\n";
        robot.automaticErrorRecovery();
        state = robot.readOnce();
        output << "After recovery: ";
        print_state(output, state);
    }
    if(state.robot_mode != franka::RobotMode::kIdle || static_cast<bool>(state.current_errors)) {
        throw std::runtime_error(startup_error(state));
    }
}

} // namespace clrc_guo::hardware

#endif
