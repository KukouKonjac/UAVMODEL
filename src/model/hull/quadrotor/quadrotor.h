#pragma once

#include "../../environment.h"
#include "../../tools/datastructure.hpp"
#include <cmath>
#include "PID.h"
#include <Eigen/Dense>

namespace uavmodel{

// 轮式车辆运动系统
class QuadrotorMoveSystem{
private:
    inline static EnvironmentInfoAgent env{};
    // void updateState(double dt, Coordinate& baseCoordinate, Hull& hull, WheelMotionParamList& params);
    // 静态 PID 控制器
    static PID pid_vx;    // 控制速度 X
    static PID pid_vy;    // 控制速度 Y
    static PID pid_z;    // 控制速度 Z
    static PID pid_phi;   // 控制姿态 roll
    static PID pid_theta; // 控制姿态 pitch
    static PID pid_psi;   // 控制偏航角 yaw
  public:
    QuadrotorMoveSystem() = default;

    static Eigen::VectorXd ode(const Eigen::VectorXd& state, QuadrotorMotionParamList& params);

    //! @param dt: 上一次调用后的时间
    //! @param baseCoordinate: 随体坐标系
    //! @param hull: 运动参数：速度、角速度、质量、转动惯量
    //! @param expectYaw: 期望方向
    //! @param expectSpeed: 期望速度
    //! @param params: 运动参数
    static void tick(
        double dt, 
        Coordinate& baseCoordinate, 
        Hull& hull, 
        double expectYaw, 
        double expectSpeed, 
        double expectHeight,
        QuadrotorMotionParamList& params
    );
};

} // namespace uavmodel