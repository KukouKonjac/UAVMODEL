#pragma once

#include "../../environment.h"
#include "../../tools/datastructure.hpp"
#include <cmath>

namespace uavmodel{

// 轮式车辆运动系统
class QuadrotorMoveSystem{
private:
    inline static EnvironmentInfoAgent env{};
    // void updateState(double dt, Coordinate& baseCoordinate, Hull& hull, WheelMotionParamList& params);
public:
    QuadrotorMoveSystem() = default;

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
        QuadrotorMotionParamList& params
    );
};
} // namespace uavmodel