#include <string>

#include "../../tools/constant.hpp"
#include "../../tools/myassert.hpp"
#include "quadrotor.h"
#include "uav_hover.h"

// 定义静态成员变量
PID uavmodel::QuadrotorMoveSystem::pid_vx(0.7, 0.0, 0.5);
PID uavmodel::QuadrotorMoveSystem::pid_vy(0.7, 0.0, 0.5);
PID uavmodel::QuadrotorMoveSystem::pid_z(0.7, 0.00001, 200);
PID uavmodel::QuadrotorMoveSystem::pid_phi(0.5, 0.0, 20.0);
PID uavmodel::QuadrotorMoveSystem::pid_theta(0.5, 0.0, 20.0);
PID uavmodel::QuadrotorMoveSystem::pid_psi(0.1, 0.0, 10.0);


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

// 各种限制导致的速度减小不会小于该速度，避免停车
constexpr inline double MIN_SPEED = 3.;


// 采用前右下坐标系，速度和转角分别控制以支持原地转方向盘、缓慢转弯、行进中变向等操作
// 由于动力学之上会采用控制器，无需考虑显式欧拉法的不稳定问题，出于简单考虑直接使用显式欧拉法

// 影响因素：
// 前轮转向：1. 最大转角 2. 最大转动速度 3. 最大侧向加速度 4. 到达目标方向时能回正
// 速度：1. 最大直线速度 2. 最大减速加速度 3. 最大侧向加速度 4. 最大前进加速度 5.
// 坡度带来的重力加速度分量

// 由于前轮转向需要时间，故速度也需要受最大侧向加速度约束
// 控制方法：符合上述约束的条件下尽可能快速地向目标调整（无人机考虑相同，后续看如何用动力学方法计算姿态角）
void QuadrotorMoveSystem::tick(double dt, Coordinate& baseCoordinate, Hull& hull, double expectYaw, double expectSpeed, double expectHeight,
                           QuadrotorMotionParamList& params) {
    double CT = params.CT, d = params.LENGTH_D, CM = params.CM, m = params.M, g = 9.8;
    uavmodel::Vector3 temp_rotation = Quaternion::fromCompressedQuaternion(baseCoordinate.attitude).getEuler();
    uavmodel::Vector3 temp_palstance = baseCoordinate.directionWorldToBody(hull.palstance);
    // 输入为期望方向、期望速度，设计pid控制器，计算出期望电机转速，输入到四旋翼动力学模型中计算新的位置、速度、姿态
    // 1.四个控制量，控制系统的输出
    // 期望速度和方向
    expectSpeed = 20;
    expectYaw = PI / 4;
    double v_ref_magnitude = expectSpeed;             // 期望速度大小
    Eigen::Vector3d v_ref_direction(cos(expectYaw), sin(expectYaw), 0); // 期望方向
    //v_ref_direction.normalize();
    Eigen::Vector3d v_ref = v_ref_direction * v_ref_magnitude;

    double psi_ref = expectYaw; // std::atan2(v_ref_direction(1), v_ref_direction(0)); // 偏航角计算

    // 速度误差计算
    Eigen::Vector3d e_v = {v_ref(0) - hull.velocity.x, v_ref(1) - hull.velocity.y, 0};//z轴不直接控制速度，控制高度
    double e_z = expectHeight + baseCoordinate.position.z;//北东地坐标系
    pid_vx.setError(e_v(0));
    pid_vy.setError(e_v(1));
    pid_z.setError(e_z);
    double ux = pid_vx.computeOutput();
    double uy = pid_vy.computeOutput();
    double uz = pid_z.computeOutput();
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
    //当前状态
    params.force = f;
    Eigen::VectorXd state(12);
    state << baseCoordinate.position.x, baseCoordinate.position.y, -1 * baseCoordinate.position.z,
        hull.velocity.x, hull.velocity.y, hull.velocity.z,
        temp_rotation.x, temp_rotation.y, temp_rotation.z, 
        temp_palstance.x, temp_palstance.y, temp_palstance.z;
    Eigen::VectorXd k1 = dt * ode(state, params);
    Eigen::VectorXd k2 = dt * ode(state + k1 / 2.0, params);
    Eigen::VectorXd k3 = dt * ode(state + k2 / 2.0, params);
    Eigen::VectorXd k4 = dt * ode(state + k3, params);

    Eigen::VectorXd new_state = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
    // 4.更新状态
    const Vector3 new_position = {new_state(0), new_state(1), -1 * new_state(2)};
    const Vector3 new_velocity = {new_state(3), new_state(4), new_state(5)};
    const Quaternion new_altitude = {new_state(6), new_state(7), new_state(8)};
    const Vector3 new_omega_body = {new_state(9), new_state(10), new_state(11)};
    const Vector3 new_palstance = baseCoordinate.directionBodyToWorld(new_omega_body);



    //// 参数计算
    //double speed = baseCoordinate.directionWorldToBody(hull.velocity).x;//前右下坐标系
    //double yaw_now = Quaternion::fromCompressedQuaternion(baseCoordinate.attitude).getEuler().z;
    //// 目标偏航角与当前偏航角的差，目标偏右为正
    //double exp_yaw_diff = angleDiff(expectYaw, yaw_now); // atan2(local_exp_direction.y, local_exp_direction.x);
    //Vector3 front_direction = baseCoordinate.directionBodyToWorld(Vector3(1., 0., 0.));
    
    baseCoordinate.position = new_position;
    baseCoordinate.attitude = new_altitude.toCompressedQuaternion();
    //// 更新速度和角速度
    hull.velocity = new_velocity;
    hull.palstance = new_palstance;

    return;
}

} // namespace uavmodel