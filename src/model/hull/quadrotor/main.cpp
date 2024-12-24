#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "UAV_Hover.h"
#include "PID.h"

int main() {

    Eigen::Vector3d target_pos(100, 100, 0);
    UAV_Hover env(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::Zero(), target_pos);

    // 控制器初始化
    PID pid_vx(0.5, 0.0, 0.5); // 控制速度 X
    PID pid_vy(0.5, 0.0, 0.5); // 控制速度 Y
    PID pid_vz(0.7, 0.0, 0.5); // 控制速度 Z

    PID pid_phi(0.5, 0.0, 20.0);   // 控制姿态 roll
    PID pid_theta(0.5, 0.0, 20.0); // 控制姿态 pitch
    PID pid_psi(0.1, 0.0, 10.0);   // 控制偏航角 yaw

    Eigen::Matrix4d inv_coe_m = env.power_allocation_mat.inverse(); // 动力分配矩阵逆矩阵

    int num = 0;

    while (num < 10) {
        // 重置环境

        while (!env.is_terminal) {
            // 期望速度和方向
            double v_ref_magnitude = 2.0;             // 期望速度大小
            Eigen::Vector3d v_ref_direction(1, 1, 0); // 期望方向
            v_ref_direction.normalize();
            Eigen::Vector3d v_ref = v_ref_direction * v_ref_magnitude;

            double psi_ref = std::atan2(v_ref_direction(1), v_ref_direction(0)); // 偏航角计算
            //psi_ref = deg2rad(psi_ref * 180.0 / M_PI);                           // 转为弧度

            // 速度误差计算
            Eigen::Vector3d e_v = v_ref - env.vel;
            pid_vx.setError(e_v(0));
            pid_vy.setError(e_v(1));
            pid_vz.setError(e_v(2));

            double ux = pid_vx.computeOutput();
            double uy = pid_vy.computeOutput();
            double uz = pid_vz.computeOutput();

            // 计算期望姿态
            double U1 = env.m * std::sqrt(ux * ux + uy * uy + (uz + env.g) * (uz + env.g));
            double phi_ref = std::asin(env.m * (ux * std::sin(psi_ref) - uy * std::cos(psi_ref)) / U1);
            double theta_ref =
                std::asin(env.m * (ux * std::cos(psi_ref) + uy * std::sin(psi_ref)) / (U1 * std::cos(phi_ref)));

            // 姿态误差计算
            double e_phi = phi_ref - env.angle(0);
            double e_theta = theta_ref - env.angle(1);
            double e_psi = psi_ref - env.angle(2);

            pid_phi.setError(e_phi);
            pid_theta.setError(e_theta);
            pid_psi.setError(e_psi);

            double U2 = pid_phi.computeOutput();
            double U3 = pid_theta.computeOutput();
            double U4 = pid_psi.computeOutput();

            // 动力分配
            Eigen::Vector4d control_input(U1, U2, U3, U4);
            Eigen::Vector4d square_omega = (inv_coe_m * control_input).cwiseMax(0); // 防止负值
            Eigen::Vector4d f = env.CT * square_omega;
            f = env.fmax * ((f.array() / env.fmax).tanh());

            // 环境更新
            env.step_update(f);
            std::cout << "Position Error: " << env.error_pos.transpose() << std::endl;
        }
        num++;
    }

    return 0;
}
