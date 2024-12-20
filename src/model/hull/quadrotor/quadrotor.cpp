#include <string>

#include "../../tools/constant.hpp"
#include "../../tools/myassert.hpp"
#include "quadrotor.h"

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
void QuadrotorMoveSystem::tick(double dt, Coordinate& baseCoordinate, Hull& hull, double expectYaw, double expectSpeed,
                           QuadrotorMotionParamList& params) {
    // 参数计算
    double speed = baseCoordinate.directionWorldToBody(hull.velocity).x;//前右下坐标系
    double yaw_now = Quaternion::fromCompressedQuaternion(baseCoordinate.attitude).getEuler().z;
    // 目标偏航角与当前偏航角的差，目标偏右为正
    double exp_yaw_diff = angleDiff(expectYaw, yaw_now); // atan2(local_exp_direction.y, local_exp_direction.x);
    Vector3 front_direction = baseCoordinate.directionBodyToWorld(Vector3(1., 0., 0.));
    
    



    const Vector3 new_position = {};
    // 计算新随体坐标系姿态四元数
   
    baseCoordinate.position = new_position;
    baseCoordinate.attitude = new_altitude.toCompressedQuaternion();
    // 更新速度和角速度
    hull.velocity = baseCoordinate.directionBodyToWorld(Vector3(new_speed, 0., 0.));
    hull.palstance = baseCoordinate.directionBodyToWorld(Vector3(0., 0., new_angular_speed));

    return;
}

} // namespace uavmodel