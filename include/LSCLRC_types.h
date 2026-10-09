#ifndef GUO_LSCLRC_TYPES_H
#define GUO_LSCLRC_TYPES_H

#include <Eigen/Dense>
#include <limits>

// 单独命名空间，允许与CLRC_2018控制器放在同一程序中进行比较。
namespace clrc_guo {

constexpr int PANDA_DOF = 7;
constexpr int PANDA_PARAMETERS = 64;
constexpr int PANDA_INERTIAL_PARAMETERS = 43;

using JointVector = Eigen::Matrix<double, PANDA_DOF, 1>;
using JointMatrix = Eigen::Matrix<double, PANDA_DOF, PANDA_DOF>;
using ParameterVector = Eigen::Matrix<double, PANDA_PARAMETERS, 1>;
using RegressorMatrix = Eigen::Matrix<double, PANDA_DOF, PANDA_PARAMETERS>;
using InformationMatrix = Eigen::Matrix<double, PANDA_PARAMETERS, PANDA_PARAMETERS>;

struct JointReference {
    JointVector q = JointVector::Zero();
    JointVector dq = JointVector::Zero();
    JointVector ddq = JointVector::Zero();
};

struct LSCLRCConfig {
    double dt = 0.001;           // 固定控制周期，滤波和历史积分使用同一周期
    double alpha = 5.0;          // 回归矩阵和力矩的共同滤波频率
    double tau_d = 3.0;          // 历史窗长度，单位s
    double ie_threshold = 1e-6;
    double pinv_tolerance = 1e-9; // 绝对截断阈值，对应MATLAB pinv(Theta,tol)
    double parameter_bound = 10.0;
    bool freeze_on_ie = true;
    bool require_full_history_window = true;

    JointMatrix K1;              // e2 = dq_ref - dq + K1*e1
    JointMatrix K2;              // tau = e1 + K2*e2 + Y*W_hat
    InformationMatrix Gamma;    // 参数更新增益
    InformationMatrix Kw;       // 最小二乘历史项增益
    JointVector torque_limit;
    JointVector torque_rate_limit;
    ParameterVector initial_W_hat = ParameterVector::Zero();

    // 对应MATLAB可选的precomputed_batch，默认使用在线历史数据。
    bool use_precomputed_batch = false;
    double publication_time = 3.0;
    InformationMatrix batch_H = InformationMatrix::Zero();
    ParameterVector batch_B = ParameterVector::Zero();

    LSCLRCConfig();
    void validate() const;
};

struct LSCLRCDiagnostic {
    JointVector e = JointVector::Zero();              // e1：位置误差
    JointVector filtered_error = JointVector::Zero(); // e2：滤波跟踪误差
    JointVector predicted_torque = JointVector::Zero();
    JointVector raw_torque = JointVector::Zero();
    JointVector acceleration_estimate = JointVector::Zero();
    JointVector tau_f = JointVector::Zero();
    RegressorMatrix Yf = RegressorMatrix::Zero();
    ParameterVector W_hat = ParameterVector::Zero(); // 本次学习更新后的参数
    ParameterVector W_dot = ParameterVector::Zero();
    ParameterVector prediction_error = ParameterVector::Zero();
    ParameterVector least_squares_term = ParameterVector::Zero(); // Theta_use^+*epsilon
    double sigma_min = 0.0;
    double best_sigma = 0.0;
    double ie_time = std::numeric_limits<double>::quiet_NaN();
    double history_fraction = 0.0;
    double direct_learning_term_norm = 0.0;
    double history_learning_term_norm = 0.0;
    int information_rank = 0;   // 与MATLAB原诊断字段一致
    int pinv_rank = 0;          // 实际学习项中未被伪逆阈值截断的方向数
    bool ie_detected = false;
    bool filter_sample_valid = false;
    bool history_ready = false;
    bool history_update_active = false;
    bool using_frozen_history = false;
};

} // namespace clrc_guo

#endif
