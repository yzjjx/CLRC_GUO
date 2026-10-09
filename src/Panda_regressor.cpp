#include "Panda_regressor.h"

#include <stdexcept>

namespace clrc_guo {

RegressorMatrix PandaRegressor::tracking_regressor(
    const JointVector& q,
    const JointVector& dq,
    const JointVector& v,
    const JointVector& dv) const
{
    const JointVector zero = JointVector::Zero();
    const RegressorMatrix Y_zero = inverse_regressor(q, zero, zero);
    const RegressorMatrix Y_mass = inverse_regressor(q, zero, dv) - Y_zero;
    const RegressorMatrix Y_plus = inverse_regressor(q, dq + v, zero);
    const RegressorMatrix Y_minus = inverse_regressor(q, dq - v, zero);
    RegressorMatrix Y = Y_zero + Y_mass + (Y_plus - Y_minus) / 4.0;

    // 最后21列为 fv、fc、fo；摩擦必须使用实际速度dq，不能使用v。
    const RegressorMatrix Y_friction = inverse_regressor(q, dq, zero);
    Y.rightCols<21>() = Y_friction.rightCols<21>();
    if(!Y.allFinite()) {
        throw std::runtime_error("Tracking regressor contains non-finite values.");
    }
    return Y;
}

} // namespace clrc_guo
