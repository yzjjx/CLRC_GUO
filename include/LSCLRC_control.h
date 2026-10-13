#ifndef GUO_LSCLRC_CONTROL_H
#define GUO_LSCLRC_CONTROL_H

#include "LSCLRC_types.h"
#include "History_window.h"
#include "Panda_regressor.h"

namespace clrc_guo {

class PandaLSCLRC {
private:
    LSCLRCConfig config;
    const PandaRegressor& dynamics;
    HistoryWindow history;
    ParameterVector W_hat;
    RegressorMatrix Yf = RegressorMatrix::Zero();
    JointVector tau_f = JointVector::Zero();
    JointVector previous_dq = JointVector::Zero();
    JointVector previous_tau = JointVector::Zero();
    bool has_previous_sample = false;
    bool ie_detected = false;
    double ie_time = std::numeric_limits<double>::quiet_NaN();
    double best_sigma = 0.0;
    double previous_time = -1.0;
    InformationMatrix Theta_e = InformationMatrix::Zero();
    InformationMatrix Theta_e_pinv = InformationMatrix::Zero();
    ParameterVector y_e = ParameterVector::Zero();
    int stored_information_rank = 0;
    int stored_pinv_rank = 0;
    double stored_sigma_min = 0.0;
    LSCLRCDiagnostic diagnostic;

    void update_filter(const JointVector& q, const JointVector& dq, double dt,
                       const JointVector& interval_torque);
    void update_history(double t, double dt);
    void store_history(const InformationMatrix& Theta, const ParameterVector& y);
    void update_parameters(const ParameterVector& raw_direction, double dt);

    // Theta是对称历史信息矩阵，特征分解实现其Moore-Penrose伪逆。
    InformationMatrix pseudo_inverse(const InformationMatrix& Theta, int& rank) const;

public:
    PandaLSCLRC(const PandaRegressor& dynamics, const LSCLRCConfig& config);
    void reset();
    double timestep() const { return config.dt; }
    const ParameterVector& parameters() const { return W_hat; }
    const LSCLRCDiagnostic& diagnostics() const { return diagnostic; }

    // 输出包含重力的总关节力矩，具体执行接口的力矩转换由调用方处理。
    JointVector compute_control(
        double t,
        const JointVector& q,
        const JointVector& dq,
        const JointReference& reference,
        double sample_dt = 0.0,
        const JointVector* interval_torque = nullptr); // 刚结束区间的平均总力矩

    // 执行端再次处理指令时，回写与辨识模型定义一致的最终施加力矩。
    void set_applied_torque(const JointVector& tau);
};

} // namespace clrc_guo

#endif
