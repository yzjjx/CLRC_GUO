#ifndef GUO_HISTORY_WINDOW_H
#define GUO_HISTORY_WINDOW_H

#include "LSCLRC_types.h"
#include <vector>

namespace clrc_guo {

class HistoryWindow {
private:
    double Ts;
    double duration;
    double elapsed = 0.0;
    int capacity;
    int count = 0;
    int next_index = 0;
    std::vector<RegressorMatrix> Yf_buffer;
    std::vector<JointVector> tau_f_buffer;
    std::vector<double> dt_buffer;
    InformationMatrix Theta = InformationMatrix::Zero();
    ParameterVector y = ParameterVector::Zero();

public:
    HistoryWindow(double dt, double duration);
    void reset();
    void step(const RegressorMatrix& Yf, const JointVector& tau_f);
    void step(const RegressorMatrix& Yf, const JointVector& tau_f, double dt);
    bool full() const { return elapsed >= duration - 1e-10 * duration; }
    double fraction() const { return elapsed / duration; }
    const InformationMatrix& information() const { return Theta; }
    const ParameterVector& torque_integral() const { return y; }
};

} // namespace clrc_guo

#endif
