#include "UAV.h"

UAV::UAV(double m, double g, double Jxx, double Jyy, double Jzz, double d, double CT, double CM, double J0, double dt)
    : m(m), g(g), Jxx(Jxx), Jyy(Jyy), Jzz(Jzz), d(d), CT(CT), CM(CM), J0(J0), dt(dt) {
    time = 0.0;
    tmax = 30.0;

    // Initialize state variables
    pos.setZero();
    vel.setZero();
    angle.setZero();
    omega_inertial.setZero();
    omega_body.setZero();

    pos_min = Eigen::Vector3d(-100, -100, -100);
    pos_max = Eigen::Vector3d(100, 100, 100);
    vel_min = Eigen::Vector3d(-10, -10, -10);
    vel_max = Eigen::Vector3d(10, 10, 10);
    angle_min = Eigen::Vector3d(-M_PI_2, -M_PI_2, -M_PI);
    angle_max = Eigen::Vector3d(M_PI_2, M_PI_2, M_PI);

    force.setZero();
    w_rotor.setZero();
    fmin = 0.0;
    fmax = 10.0;

    is_terminal = false;
    terminal_flag = 0;

    power_allocation_mat.resize(4, 4);
    power_allocation_mat << CT, CT, CT, CT, CT * d / sqrt(2), -CT * d / sqrt(2), -CT * d / sqrt(2), CT * d / sqrt(2),
        -CT * d / sqrt(2), -CT * d / sqrt(2), CT * d / sqrt(2), CT * d / sqrt(2), -CM, CM, -CM, CM;
}

void UAV::f2omega() {
    for (int i = 0; i < 4; ++i) {
        w_rotor(i) = sqrt(force(i) / CT);
    }
}

bool UAV::is_out() const {
    if ((angle.array() > angle_max.array()).any() || (angle.array() < angle_min.array()).any()) {
        std::cout << "Attitude out..." << std::endl;
        return true;
    }
    if ((pos.array() > pos_max.array()).any() || (pos.array() < pos_min.array()).any()) {
        std::cout << "Position out..." << std::endl;
        return true;
    }
    return false;
}

Eigen::VectorXd UAV::ode(const Eigen::VectorXd& state) {
    Eigen::VectorXd dx(12);

    double _f = force.sum();
    f2omega();

    dx.head<3>() = state.segment<3>(3); // Velocity
    dx.segment<3>(3) << _f / m * (cos(state(8)) * sin(state(7)) * cos(state(6)) + sin(state(8)) * sin(state(6))),
        _f / m * (sin(state(8)) * sin(state(7)) * cos(state(6)) - cos(state(8)) * sin(state(6))),
        -g + _f / m * cos(state(6)) * cos(state(7));

    // Angular velocity differential
    dx.segment<3>(9) << 0, 0, 0; // Placeholder

    return dx;
}

void UAV::rk44(const Eigen::VectorXd& action) {
    force = action;
    double h = dt;

    Eigen::VectorXd state(12);
    state << pos, vel, angle, omega_body;

    Eigen::VectorXd k1 = h * ode(state);
    Eigen::VectorXd k2 = h * ode(state + k1 / 2);
    Eigen::VectorXd k3 = h * ode(state + k2 / 2);
    Eigen::VectorXd k4 = h * ode(state + k3);

    Eigen::VectorXd new_state = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6;

    pos = new_state.segment<3>(0);
    vel = new_state.segment<3>(3);
    angle = new_state.segment<3>(6);
    omega_body = new_state.segment<3>(9);

    time += h;
}
