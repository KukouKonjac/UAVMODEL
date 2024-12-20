#include "PID.h"
#include <cmath>

PID::PID(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), e(0), e_diff(0), e_int(0), p_out(0), i_out(0), d_out(0), dead_zone(0), seg_linear(false),
      fuzzy(false), neural(false) {}

void PID::setPID(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void PID::setMultiPID() {
    // Placeholder for multi-PID logic
}

void PID::setError(double _e) {
    e_diff = _e - e;
    e = _e;
    e_int += _e;
}

double PID::computeOutput() {
    if (std::fabs(e) >= dead_zone) {
        p_out = kp * e;
        i_out = ki * e_int;
        d_out = kd * e_diff;
    } else {
        p_out = i_out = d_out = 0.0;
    }
    return p_out + i_out + d_out;
}

void PID::reset() { *this = PID(kp, ki, kd); }
