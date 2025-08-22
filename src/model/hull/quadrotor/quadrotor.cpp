#include <string>

#include "../../tools/constant.hpp"
#include "../../tools/myassert.hpp"
#include "quadrotor.h"
#include "uav_hover.h"

// 定义静态成员变量
PID uavmodel::QuadrotorMoveSystem::pid_vx(0.5, 0.0000, 0);
PID uavmodel::QuadrotorMoveSystem::pid_vy(0.5, 0.0000, 0);
PID uavmodel::QuadrotorMoveSystem::pid_z(1, 0.0, 50);
PID uavmodel::QuadrotorMoveSystem::pid_phi(0.5, 0.01, 5.0);
PID uavmodel::QuadrotorMoveSystem::pid_theta(0.5, 0.01, 5.0);
PID uavmodel::QuadrotorMoveSystem::pid_psi(0.1, 0.0, 10.0);

double uavmodel::QuadrotorMoveSystem::height_pid_integral = 0;
double uavmodel::QuadrotorMoveSystem::height_pid_last_error = 0;

namespace {

using namespace uavmodel;

// 投射到给定范围内
double clamp(double value, double min, double max) {
    if (value < min)
        value = min;
    if (value > max)
        value = max;
    return value;
}

// force inline?
double clamp(double value, double bound) {
    bound = fabs(bound);
    // if (bound < 0)
    //     bound = -bound;
    if (value < -bound)
        value = -bound;
    if (value > bound)
        value = bound;
    return value;
}

double softClamp(double value, double bound) { return atan(value * (PI / 2.) / bound) / (PI / 2.) * bound; }

double softClamp(double value, double min, double max) {
    double mid = (min + max) / 2;
    return softClamp(value - mid, max - mid) + mid;
}

// 获取当前转角时角速度与速度比例
double angularLinearRatio(double rad, double LENGTH) { return tan(rad) / LENGTH; }

// 获取符号
int sign(double value) {
    if (value > INF_SMALL)
        return 1;
    if (value < -INF_SMALL)
        return -1;
    return 0;
}

double angleDiff(double angleExpect, double angleNow) {
    double tmp = angleExpect - angleNow;
    if (tmp > PI)
        tmp -= 2 * PI;
    if (tmp < -PI)
        tmp += 2 * PI;
    return tmp;
}

bool equal(double a, double b) { return fabs(a - b) < INF_SMALL; }

}; // namespace

namespace uavmodel {

// 求微分
Eigen::VectorXd QuadrotorMoveSystem::ode(const Eigen::VectorXd& state, QuadrotorMotionParamList& params) {
    // 状态向量 state 包含 12 个元素：位置 (x, y, z)，速度 (vx, vy, vz)，姿态角 (roll, pitch, yaw)，角速度 (p, q, r)
    double CT = params.CT;
    double CM = params.CM;
    double d = params.LENGTH_D;
    Eigen::Vector4d force = params.force;
    Eigen::Vector4d w_rotor = params.w_rotor;
    Eigen::VectorXd dx(12);
    double Jxx = params.JXX;
    double Jyy = params.JYY;
    double Jzz = params.JZZ;
    double J0 = params.J0;
    double m = params.M;
    // 从状态向量中提取变量
    double _x = state(0), _y = state(1), _z = state(2);
    double _vx = state(3), _vy = state(4), _vz = state(5);
    double _phi = state(6), _theta = state(7), _psi = state(8);
    double _p = state(9), _q = state(10), _r = state(11);

    double _f = force.sum();

    for (int i = 0; i < 4; ++i) {
        w_rotor(i) = sqrt(force(i) / CT);
    }

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
    double dvz = -9.8 + _f / m * cos(_phi) * cos(_theta);

    // 填充导数向量
    dx.segment<3>(0) << _vx, _vy, _vz; // 位置导数（速度）
    dx.segment<3>(3) << dvx, dvy, dvz; // 速度导数
    dx.segment<3>(6) << d_angles;      // 姿态角导数
    dx.segment<3>(9) << dp, dq, dr;    // 角速度导数

    return dx;
}

// 采用前右下坐标系，速度和转角分别控制以支持原地转方向盘、缓慢转弯、行进中变向等操作
// 由于动力学之上会采用控制器，无需考虑显式欧拉法的不稳定问题，出于简单考虑直接使用显式欧拉法（无人机采用四阶龙格库塔法）

// 控制方法：符合上述约束的条件下尽可能快速地向目标调整（无人机pid控制，用动力学方法计算姿态角）
void QuadrotorMoveSystem::tick(double dt, Coordinate& baseCoordinate, Hull& hull, double expectYaw, double expectSpeed,
                               double expectHeight, QuadrotorMotionParamList& params) {
    double CT = params.CT, d = params.LENGTH_D, CM = params.CM, m = params.M, g = 9.8;
    uavmodel::Vector3 temp_rotation = Quaternion::fromCompressedQuaternion(baseCoordinate.attitude).getEuler();
    uavmodel::Vector3 temp_palstance = baseCoordinate.directionWorldToBody(hull.palstance);
    // 输入为期望方向、期望速度，设计pid控制器，计算出期望电机转速，输入到四旋翼动力学模型中计算新的位置、速度、姿态
    // 1.四个控制量，控制系统的输出
    // 期望速度和方向
    double speed_restriction_level = params.MAX_LEVELFLY_SPEED; // 最大平飞速度限制

    double v_ref_magnitude = clamp(expectSpeed, speed_restriction_level); // 期望速度大小
    Eigen::Vector3d v_ref_direction(cos(expectYaw), sin(expectYaw), 0);   // 期望方向
    // v_ref_direction.normalize();
    Eigen::Vector3d v_ref = v_ref_direction * v_ref_magnitude;
    double psi_ref = expectYaw; // std::atan2(v_ref_direction(1), v_ref_direction(0)); // 偏航角计算
    // 速度误差计算
    Eigen::Vector3d e_v = {v_ref(0) - hull.velocity.x, v_ref(1) - hull.velocity.y, 0}; // z轴不直接控制速度，控制高度
    double e_z = std::min(expectHeight, params.MAX_CLIMB_HEIGHT) + baseCoordinate.position.z; // 北东地坐标系
    pid_vx.setError(e_v(0));
    pid_vy.setError(e_v(1));
    pid_z.setError(e_z);
    double ux = pid_vx.computeOutput();
    double uy = pid_vy.computeOutput();
    double uz = pid_z.computeOutput();
    uz = std::max((-1 * params.MAX_DIVE_SPEED + hull.velocity.z) / dt,
                  std::min((params.MAX_CLIMB_SPEED + hull.velocity.z) / dt, uz));
    // 计算期望姿态
    double U1 = m * std::sqrt(ux * ux + uy * uy + (uz + g) * (uz + g));
    double phi_ref = std::asin(m * (ux * std::sin(psi_ref) - uy * std::cos(psi_ref)) / U1);
    double theta_ref = std::asin(m * (ux * std::cos(psi_ref) + uy * std::sin(psi_ref)) / (U1 * std::cos(phi_ref)));
    // 姿态误差计算
    double e_phi = phi_ref - temp_rotation.x;
    double e_theta = theta_ref - temp_rotation.y;
    double e_psi = psi_ref - temp_rotation.z;
    pid_phi.setError(e_phi);
    pid_theta.setError(e_theta);
    pid_psi.setError(e_psi);

    double U2 = pid_phi.computeOutput();
    double U3 = pid_theta.computeOutput();
    double U4 = pid_psi.computeOutput();
    // 2.动力分配矩阵，后续在序列化里直接计算一次，这里会一直计算
    Eigen::MatrixXd power_allocation_mat(4, 4);
    power_allocation_mat << CT, CT, CT, CT, CT * d / sqrt(2), -CT * d / sqrt(2), -CT * d / sqrt(2), CT * d / sqrt(2),
        -CT * d / sqrt(2), -CT * d / sqrt(2), CT * d / sqrt(2), CT * d / sqrt(2), -CM, CM, -CM, CM;
    Eigen::MatrixXd inv_coe_m = power_allocation_mat.inverse();
    Eigen::Vector4d control_input(U1, U2, U3, U4);
    Eigen::Vector4d square_omega = (inv_coe_m * control_input).cwiseMax(0); // 防止负值
    Eigen::Vector4d f = CT * square_omega;
    f = params.F_MAX * ((f.array() / params.F_MAX).tanh());
    // 3.四阶龙格库塔法计算新的位置、速度、姿态
    // 当前状态
    params.force = f;
    Eigen::VectorXd state(12);
    state << baseCoordinate.position.x, baseCoordinate.position.y, -1 * baseCoordinate.position.z, hull.velocity.x,
        hull.velocity.y, -1 * hull.velocity.z, temp_rotation.x, temp_rotation.y, temp_rotation.z, temp_palstance.x,
        temp_palstance.y, temp_palstance.z;
    Eigen::VectorXd k1 = dt * ode(state, params);
    Eigen::VectorXd k2 = dt * ode(state + k1 / 2.0, params);
    Eigen::VectorXd k3 = dt * ode(state + k2 / 2.0, params);
    Eigen::VectorXd k4 = dt * ode(state + k3, params);

    Eigen::VectorXd new_state = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
    // 4.更新状态
    const Vector3 new_position = {new_state(0), new_state(1), -1 * new_state(2)};
    const Vector3 new_velocity = {new_state(3), new_state(4), -1 * new_state(5)};
    const Quaternion new_altitude = {new_state(6), new_state(7), new_state(8)};
    const Vector3 new_omega_body = {new_state(9), new_state(10), new_state(11)};
    const Vector3 new_palstance = baseCoordinate.directionBodyToWorld(new_omega_body);

    baseCoordinate.position = new_position;
    baseCoordinate.attitude = new_altitude.toCompressedQuaternion();
    //// 更新速度和角速度
    hull.velocity = new_velocity;
    hull.palstance = new_palstance;
    params.BATTERY -= dt;
    return;
}

void QuadrotorMoveSystem::tickspecific(double dt, Coordinate& baseCoordinate, Hull& hull, double expectYaw,
                                       double expectSpeed, double expectHeight, QuadrotorMotionParamList& params) {
    double m = params.M;
    double g = 9.8;
    double psi_ref = expectYaw;
    uavmodel::Vector3 temp_rotation = Quaternion::fromCompressedQuaternion(baseCoordinate.attitude).getEuler();
    const double MAX_LINEAR_ACC = 10;                                     // 最大线加速度(m/s²)
    uavmodel::Vector3 v_ref_direction(cos(expectYaw), sin(expectYaw), 0); // 期望方向
    // v_ref_direction.normalize();
    uavmodel::Vector3 v_ref = v_ref_direction * std::min(expectSpeed, params.MAX_LEVELFLY_SPEED);
    // 1. 计算期望加速度（速度控制）
    double target_z = -std::min(expectHeight, params.MAX_CLIMB_HEIGHT);
    double height_error = baseCoordinate.position.z - target_z;

    // PID 参数（可调）
    double Kp = 1.5;
    double Ki = 0.0;
    double Kd = 3.0;

    // 积分项更新
    height_pid_integral += height_error * dt;
    // 防止积分饱和（可调）
    height_pid_integral = std::clamp(height_pid_integral, -2.0, 2.0);
    double derivative = (dt > 1e-6) ? (height_error - height_pid_last_error) / dt : 0.0;

    double acc_z_desired = -(Kp * height_error + Ki * height_pid_integral + Kd * derivative);
    /*std::cout << Kp * height_error << " " << Ki * height_pid_integral << " " << Kd * derivative << " ";*/
    acc_z_desired = std::clamp(acc_z_desired, -MAX_LINEAR_ACC, MAX_LINEAR_ACC);

    height_pid_last_error = height_error; // 更新上一次误差

    uavmodel::Vector3 target_acc{0, 0, 0};
    uavmodel::Vector3 vel_error = v_ref - hull.velocity;
    double acc_magnitude_xy = std::min(vel_error.norm() / dt, MAX_LINEAR_ACC);
    if (vel_error.norm() > 1e-6) {
        uavmodel::Vector3 dir = vel_error.normalize();
        target_acc.x = dir.x * acc_magnitude_xy;
        target_acc.y = dir.y * acc_magnitude_xy;
    }
    target_acc.z = acc_z_desired;

    // 2. 计算期望角速度（方向控制）
    double U1 = m * std::sqrt(target_acc.x * target_acc.x + target_acc.y * target_acc.y +
                              (target_acc.z + g) * (target_acc.z + g));

    double phi_ref = 0.0, theta_ref = 0.0;
    if (U1 > 1e-6) {
        double sin_phi = m * (target_acc.x * std::sin(psi_ref) - target_acc.y * std::cos(psi_ref)) / U1;
        sin_phi = std::clamp(sin_phi, -0.999, 0.999);
        phi_ref = std::asin(sin_phi);

        double cos_phi = std::cos(phi_ref);
        if (std::abs(cos_phi) > 1e-6) {
            double sin_theta =
                m * (target_acc.x * std::cos(psi_ref) + target_acc.y * std::sin(psi_ref)) / (U1 * cos_phi);
            sin_theta = std::clamp(sin_theta, -0.999, 0.999);
            theta_ref = std::asin(sin_theta);
        }
    }

    // 3. 角速度限制
    uavmodel::Vector3 angular_vel = {
        clamp((phi_ref - temp_rotation.x) / dt, -params.ROTATE_SPEED, params.ROTATE_SPEED),
        clamp((theta_ref - temp_rotation.y) / dt, -params.ROTATE_SPEED, params.ROTATE_SPEED),
        clamp((psi_ref - temp_rotation.z) / dt, -params.ROTATE_SPEED, params.ROTATE_SPEED)};
    // 3. 直接更新状态（简化动力学）
    // 超出最大飞行时间时失去控制
    if (params.BATTERY >= 0) {
        hull.velocity += target_acc * dt;
        hull.velocity.z = std::clamp(hull.velocity.z, -params.MAX_CLIMB_SPEED, params.MAX_CLIMB_SPEED);
        // std::cout << target_acc.z <<" "<< hull.velocity.z << std::endl;
        hull.palstance = angular_vel;
        baseCoordinate.position += hull.velocity * dt;
    } else {
        hull.velocity.x = 0;
        hull.velocity.y = 0;
        hull.velocity.z += g * dt;
        baseCoordinate.position += hull.velocity * dt;
        if (baseCoordinate.position.z >= 0.0) {
            baseCoordinate.position.z = 0.0;
            hull.velocity.z = 0.0;
        }
    }
    const Quaternion new_attitude = {temp_rotation.x + hull.palstance.x * dt, temp_rotation.y + hull.palstance.y * dt,
                                     temp_rotation.z + hull.palstance.z * dt};
    baseCoordinate.attitude = new_attitude.toCompressedQuaternion();
    //// 更新速度和角速度
    params.BATTERY -= dt;
    return;
}

} // namespace uavmodel