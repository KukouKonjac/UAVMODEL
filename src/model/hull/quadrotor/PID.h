#ifndef PID_H
#define PID_H

class PID {
  private:
    double kp, ki, kd;          // PID coefficients
    double e, e_diff, e_int;    // Error terms
    double p_out, i_out, d_out; // Output terms
    double dead_zone;

  public:
    PID(double kp = 0, double ki = 0, double kd = 0);
    void setPID(double _kp, double _ki, double _kd);
    void setMultiPID();
    void setError(double _e);
    double computeOutput();
    void reset();
};

#endif // PID_H
