#ifndef UAV_H
#define UAV_H

#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

class UAV {
  public:
    UAV(double m = 0.8, double g = 9.8, double Jxx = 4.212e-3, double Jyy = 4.212e-3, double Jzz = 8.255e-3,
        double d = 0.12, double CT = 2.168e-6, double CM = 2.136e-8, double J0 = 1.01e-5, double dt = 0.01);

    virtual ~UAV() {}

    bool is_out() const;
    void rk44(const Eigen::VectorXd& action);
    virtual Eigen::VectorXd ode(const Eigen::VectorXd& state);

  protected:
    // UAV physical parameters
    double m, g, Jxx, Jyy, Jzz, d, CT, CM, J0, dt;
    double time, tmax;
    Eigen::MatrixXd power_allocation_mat;

    // State variables
    Eigen::Vector3d pos, vel, angle, omega_inertial, omega_body;
    Eigen::Vector3d pos_min, pos_max, vel_min, vel_max, angle_min, angle_max;
    Eigen::VectorXd control_state;

    Eigen::Vector4d force; // Control inputs
    Eigen::Vector4d w_rotor;
    double fmin, fmax;

    // Terminal state
    bool is_terminal;
    int terminal_flag;

    void f2omega();
};

#endif // UAV_H
