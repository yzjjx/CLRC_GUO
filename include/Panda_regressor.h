#ifndef GUO_PANDA_REGRESSOR_H
#define GUO_PANDA_REGRESSOR_H

#include "LSCLRC_types.h"

namespace clrc_guo {

class PandaRegressor {
public:
    // Yb(q,dq,ddq)：由 MATLAB 数值核逐项转换，列顺序保持不变。
    RegressorMatrix inverse_regressor(
        const JointVector& q,
        const JointVector& dq,
        const JointVector& ddq) const;

    // Y(q,dq,v,dv)：控制律需要 M(q)*dv + C(q,dq)*v + g(q) + friction(dq)。
    RegressorMatrix tracking_regressor(
        const JointVector& q,
        const JointVector& dq,
        const JointVector& v,
        const JointVector& dv) const;

    // 与回归列配对的名义参数，可用于指定估计初值或进行参数对照。
    static ParameterVector nominal_parameters();
};

} // namespace clrc_guo

#endif
