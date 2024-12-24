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
    angle_min = Eigen::Vector3d(deg2rad(-80), deg2rad(-80), deg2rad(-180));
    angle_max = Eigen::Vector3d(deg2rad(80), deg2rad(80), deg2rad(180));
    dangle_min = Eigen::Vector3d(deg2rad(-360 * 3), deg2rad(-360 * 3), deg2rad(-360 * 2));
    dangle_max = Eigen::Vector3d(deg2rad(360 * 3), deg2rad(360 * 3), deg2rad(360 * 2));


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
    // 误差范围
    const double angle_tolerance = deg2rad(2.0); // 容许角度误差
    const double pos_tolerance = 1e-2;           // 容许位置误差

    // 检查角度是否超出范围（考虑误差）
    if ((angle.array() > (angle_max.array() + angle_tolerance)).any() ||
        (angle.array() < (angle_min.array() - angle_tolerance)).any()) {
        std::cout << "Attitude out..." << std::endl;
        return true;
    }

    // 检查位置是否超出范围（考虑误差）
    if ((pos.array() > (pos_max.array() + pos_tolerance)).any() ||
        (pos.array() < (pos_min.array() - pos_tolerance)).any()) {
        std::cout << "Position out..." << std::endl;
        return true;
    }

    return false;
}


Eigen::VectorXd UAV::ode(const Eigen::VectorXd& state) {
    Eigen::VectorXd dx(12);

    // 从状态向量中提取变量
    double _x = state(0), _y = state(1), _z = state(2);
    double _vx = state(3), _vy = state(4), _vz = state(5);
    double _phi = state(6), _theta = state(7), _psi = state(8);
    double _p = state(9), _q = state(10), _r = state(11);

    double _f = force.sum();
    f2omega();

    Eigen::Vector4d square_w = w_rotor.array().square();

    // 1. 无人机绕机体系旋转的角速度 p, q, r 的微分方程
    double dp = (CT * d / sqrt(2.0) * (square_w.dot(Eigen::Vector4d(1, -1, -1, 1))) + (Jyy - Jzz) * _q * _r -
                 J0 * _q * (w_rotor(0) - w_rotor(1) + w_rotor(2) - w_rotor(3))) /
                Jxx;

    double dq = (CT * d / sqrt(2.0) * (square_w.dot(Eigen::Vector4d(-1, -1, 1, 1))) + (Jzz - Jxx) * _p * _r -
                 J0 * _p * (-w_rotor(0) + w_rotor(1) - w_rotor(2) + w_rotor(3))) /
                Jyy;

    double dr = (CM * (square_w.dot(Eigen::Vector4d(-1, 1, -1, 1))) + (Jxx - Jyy) * _p * _q) / Jzz;

    // 2. 无人机在惯性系下的姿态角 phi, theta, psi 的微分方程
    Eigen::Matrix3d R_pqr2diner;
    R_pqr2diner << 1, tan(_theta) * sin(_phi), tan(_theta) * cos(_phi), 0, cos(_phi), -sin(_phi), 0,
        sin(_phi) / cos(_theta), cos(_phi) / cos(_theta);

    Eigen::Vector3d d_angles = R_pqr2diner * Eigen::Vector3d(_p, _q, _r);

    // 3. 无人机在惯性系下的位置和速度的微分方程
    double dvx = _f / m * (cos(_psi) * sin(_theta) * cos(_phi) + sin(_psi) * sin(_phi));
    double dvy = _f / m * (sin(_psi) * sin(_theta) * cos(_phi) - cos(_psi) * sin(_phi));
    double dvz = -g + _f / m * cos(_phi) * cos(_theta);

    // 填充导数向量
    dx.segment<3>(0) << _vx, _vy, _vz; // 位置导数（速度）
    dx.segment<3>(3) << dvx, dvy, dvz; // 速度导数
    dx.segment<3>(6) << d_angles;      // 姿态角导数
    dx.segment<3>(9) << dp, dq, dr;    // 角速度导数

    return dx;
}

void UAV::rk44(const Eigen::VectorXd& action) {
    // 更新推力
    force = action;
    double h = dt; // 时间步长
    double tt = time + dt;

    // 主循环，用于细化步长（如果需要）
    while (time < tt) {
        // 当前状态
        Eigen::VectorXd state(12);
        state << pos, vel, angle, omega_body;

        // Runge-Kutta 4阶积分
        Eigen::VectorXd k1 = h * ode(state);
        Eigen::VectorXd k2 = h * ode(state + k1 / 2.0);
        Eigen::VectorXd k3 = h * ode(state + k2 / 2.0);
        Eigen::VectorXd k4 = h * ode(state + k3);

        Eigen::VectorXd new_state = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;

        // 更新状态
        pos = new_state.segment<3>(0);        // 更新位置
        vel = new_state.segment<3>(3);        // 更新速度
        angle = new_state.segment<3>(6);      // 更新姿态角
        omega_body = new_state.segment<3>(9); // 更新机体系角速度

        time += h;
    }

    // 计算惯性系下的角速度
    Eigen::Matrix3d R_pqr2diner;
    double phi = angle(0), theta = angle(1), psi = angle(2);
    R_pqr2diner << 1, tan(theta) * sin(phi), tan(theta) * cos(phi), 0, cos(phi), -sin(phi), 0, sin(phi) / cos(theta),
        cos(phi) / cos(theta);

    omega_inertial = R_pqr2diner * omega_body;

    // 限制偏航角 psi 的范围到 [-π, π]
    if (angle(2) > M_PI) {
        angle(2) -= 2 * M_PI;
    }
    if (angle(2) < -M_PI) {
        angle(2) += 2 * M_PI;
    }
}
