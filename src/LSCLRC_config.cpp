#include "LSCLRC_types.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace clrc_guo {

LSCLRCConfig::LSCLRCConfig()
{
    // 对应当前make_default_config('franka','tuned').lsclrc2020。
    JointVector k2_diag;
    k2_diag << 15, 15, 15, 15, 6, 6, 3.75;
    K1 = 6.0 * JointMatrix::Identity();
    K2 = k2_diag.asDiagonal();
    Gamma = 0.3 * InformationMatrix::Identity();
    Kw = 2.5 * InformationMatrix::Identity();

    // MATLAB的LS配置不限制力矩；默认禁用额外的接口限速。
    // 调用方可在构造控制器前配置正的有限限制。
    torque_limit.setConstant(std::numeric_limits<double>::infinity());
    torque_rate_limit.setConstant(std::numeric_limits<double>::infinity());
}

void LSCLRCConfig::validate() const
{
    if(!std::isfinite(dt) || dt <= 0 || !std::isfinite(tau_d) || tau_d <= 0
        || tau_d / dt > 1000000 || !std::isfinite(alpha) || alpha <= 0
        || !std::isfinite(ie_threshold) || ie_threshold <= 0
        || !std::isfinite(pinv_tolerance) || pinv_tolerance < 0
        || std::isnan(parameter_bound) || parameter_bound <= 0) {
        throw std::invalid_argument("LS-CLRC scalar configuration is invalid.");
    }
    if(!K1.allFinite() || !K2.allFinite() || !Gamma.allFinite() || !Kw.allFinite()
        || !K1.isApprox(K1.transpose()) || !K2.isApprox(K2.transpose())
        || !Gamma.isApprox(Gamma.transpose()) || !Kw.isApprox(Kw.transpose())
        || K1.llt().info() != Eigen::Success || K2.llt().info() != Eigen::Success
        || Gamma.llt().info() != Eigen::Success || Kw.llt().info() != Eigen::Success) {
        throw std::invalid_argument("K1, K2, Gamma and Kw must be symmetric positive definite.");
    }
    if(!initial_W_hat.allFinite() || initial_W_hat.norm() > parameter_bound) {
        throw std::invalid_argument("Initial parameter estimate is invalid or outside the projection bound.");
    }
    for(int joint = 0; joint < PANDA_DOF; joint++) {
        if(std::isnan(torque_limit(joint)) || torque_limit(joint) <= 0
            || std::isnan(torque_rate_limit(joint)) || torque_rate_limit(joint) <= 0) {
            throw std::invalid_argument("Torque limits must be positive; positive infinity disables a limit.");
        }
    }
    if(use_precomputed_batch) {
        if(!std::isfinite(publication_time) || publication_time < 0
            || !batch_H.allFinite() || !batch_B.allFinite()
            || !batch_H.isApprox(batch_H.transpose())) {
            throw std::invalid_argument("Precomputed history batch is invalid.");
        }
        const InformationMatrix symmetric_H = 0.5 * (batch_H + batch_H.transpose());
        Eigen::SelfAdjointEigenSolver<InformationMatrix> solver(symmetric_H, Eigen::EigenvaluesOnly);
        if(solver.info() != Eigen::Success
            || solver.eigenvalues().minCoeff() < -1e-10 * std::max(1.0, batch_H.norm())) {
            throw std::invalid_argument("Precomputed information matrix must be positive semidefinite.");
        }
    }
}

} // namespace clrc_guo
