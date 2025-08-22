#include <cmath>
#include <set>
#include <tuple>

#include "../tools/vector3.hpp"
#include "quadrotor/quadrotor.h"
#include "uavhull.h"
#include "wheel/wheel.h"

#ifndef M_PI
#define M_PI 3.14159
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.0)
#endif

namespace {

using namespace uavmodel::command;

constexpr size_t validMovingCommandMask =
    size_t(1) << static_cast<int>(COMMAND_TYPE::FOLLOWCAR) | size_t(1) << static_cast<int>(COMMAND_TYPE::CLIMB) |
    size_t(1) << static_cast<int>(COMMAND_TYPE::DIVE) | size_t(1) << static_cast<int>(COMMAND_TYPE::LEVELFLIGHT) |
    size_t(1) << static_cast<int>(COMMAND_TYPE::HOVER) | size_t(1) << static_cast<int>(COMMAND_TYPE::BACK) |
    size_t(1) << static_cast<int>(COMMAND_TYPE::SURROUND);

}; // namespace

namespace uavmodel {

void HullSystem::tick(double dt, Components& c) {
    using namespace std;
    auto& damage = c.getSpecificSingleton<DamageModel>().value();
    if (damage.damageLevel == DAMAGE_LEVEL::K || damage.damageLevel == DAMAGE_LEVEL::KK) {
        double& repairTime = (damage.damageLevel == DAMAGE_LEVEL::K) ? damage.repairTime_K : damage.repairTime_KK;
        repairTime -= dt;
        if (repairTime <= 0) {
            damage.damageLevel = DAMAGE_LEVEL::N;
        }
        return;
    }

    auto& optParam = c.getSpecificSingleton<QuadrotorMotionParamList>();
    if (!optParam.has_value()) {
        return;
    }

    auto& memberscan = c.getSpecificSingleton<SystemScannedMemory>().value();
    for (auto& [k, v] : memberscan) {
        if (/*get<1>(v).baseInfo.type == uavmodel::BaseInfo::ENTITY_TYPE::SUPPORTCAR &&*/
            get<1>(v).baseInfo.side == c.getSpecificSingleton<SID>().value() &&
            c.getSpecificSingleton<PLATOONID>().value() == get<1>(v).baseInfo.platoonid) {
            optParam.value().TIED_WITH_CAR = k;
            break;
        }
    }
    if (flyflag == false) {
        if (optParam.value().TIED_WITH_CAR == -1) {
            ReleasePosition = c.getSpecificSingleton<Coordinate>().value().position;
        } else {
            ReleasePosition = get<1>(memberscan[optParam.value().TIED_WITH_CAR]).position;
        }
    } else {
        if (optParam.value().TIED_WITH_CAR != -1) {
            ReleasePosition = get<1>(memberscan[optParam.value().TIED_WITH_CAR]).position;
        }
    }

    auto& param = optParam.value();
    Coordinate coordinate = c.getSpecificSingleton<Coordinate>().value();
    for (auto&& [k, v] : c.getSpecificSingleton<CommandBuffer>().value()) {
        if ((validMovingCommandMask & size_t(1) << static_cast<int>(k)) == 0) {
            continue;
        }
        auto [param1, param2] = any_cast<tuple<double, double>>(v);
        if (k == COMMAND_TYPE::CLIMB || k == COMMAND_TYPE::DIVE) {
            height = param1;
            flyflag = true;
        } else if (k == COMMAND_TYPE::LEVELFLIGHT) {
            height = -c.getSpecificSingleton<Coordinate>().value().position.z;
            speed = param1;
            direction = param2;
        } else if (k == COMMAND_TYPE::HOVER) {
            speed = 0;
        } else if (k == COMMAND_TYPE::BACK) {
            auto& tar_pos = std::get<1>(c.getSpecificSingleton<SystemScannedMemory>().value()[param1]).position;
            auto& tar_vel = std::get<1>(c.getSpecificSingleton<SystemScannedMemory>().value()[param1]).velocity;
            // if (abs(tar.z - coordinate.position.z) < 1) {
            //     flyflag = false;
            //     continue;
            // }
            // double dis = sqrt(pow((c.getSpecificSingleton<Coordinate>().value().position.x - tar.x), 2) +
            // pow((c.getSpecificSingleton<Coordinate>().value().position.y - tar.y), 2)); speed = dis > 200 ? /*20 :
            // floor(dis / 20)*/ speed : floor(dis / 20); speed = speed < 0 ? 0 : speed; if (speed == 0) {
            //     height = -1 * tar.z;
            // }
            // direction = atan2(tar.y - c.getSpecificSingleton<Coordinate>().value().position.y,
            //                   tar.x - c.getSpecificSingleton<Coordinate>().value().position.x);

            const double target_x = tar_pos.x;
            const double target_y = tar_pos.y;
            const double target_z = -tar_pos.z; // 无人机目标高度

            const auto& cur_pos = c.getSpecificSingleton<Coordinate>().value().position;
            const auto& cur_vel = c.getSpecificSingleton<Hull>().value().velocity;
            double dx = target_x - cur_pos.x;
            double dy = target_y - cur_pos.y;
            double dz = target_z + cur_pos.z;

            double horizontal_dist = std::sqrt(dx * dx + dy * dy);

            double desired_vx = tar_vel.x + 1.0 * dx; // P 控制：位置误差 → 附加速度
            double desired_vy = tar_vel.y + 1.0 * dy;

            // 限制最大速度
            double max_speed = optParam.value().MAX_LEVELFLY_SPEED;
            double desired_speed = std::sqrt(desired_vx * desired_vx + desired_vy * desired_vy);
            if (desired_speed > max_speed) {
                double scale = max_speed / desired_speed;
                desired_vx *= scale;
                desired_vy *= scale;
            }
            // 输出给 tickspecific
            speed = desired_speed;                          // 用于控制油门
            direction = std::atan2(desired_vy, desired_vx); // 期望速度方向

            height = target_z; // 保持目标高度
            if (horizontal_dist < 5.0) {
                height = target_z * (horizontal_dist / 5.0); // 从当前高度缓降到 0
            }
            if ((tar_pos - cur_pos).norm() < 1e-2 && (cur_vel - tar_vel).norm() < 1e-2) {
                flyflag = false;
            }

        } else if (k == COMMAND_TYPE::SURROUND) {
            const auto& cur_pos = c.getSpecificSingleton<Coordinate>().value().position;
            const auto& params = c.getSpecificSingleton<QuadrotorMotionParamList>().value();
            auto& hull = c.getSpecificSingleton<Hull>().value();
            auto& state = c.getSpecificSingleton<SurroundState>().value();

            const double R_SMALL = 500.0; // 小轨道半径（小于 param1）
            const double R_MULTIPLIER = 2.0;
            const double finalRadius = R_SMALL * R_MULTIPLIER; // 外圈目标半径
            const double EXPAND_RATE = speed;                  // 扩展速度（m/s），可调整
            const double omega = 0.1;                          // 角速度 (rad/s)

            const double ORBIT_TOLERANCE = 30.0; // 允许误差：±50m

            // --- 第一次初始化：计算圆心，设置状态 ---
            if (!state.isSurrounding) {
                state.isSurrounding = true;

                // 圆心：仍使用 param1 和 param2（固定点）
                state.centerPosition.x = cur_pos.x + param1 * std::cos(param2);
                state.centerPosition.y = cur_pos.y + param1 * std::sin(param2);
                state.centerPosition.z = cur_pos.z;
                std::cout << state.centerPosition.x << endl;

                // 初始化角度（从当前位置指向圆心）
                const double dx = cur_pos.x - state.centerPosition.x;
                const double dy = cur_pos.y - state.centerPosition.y;
                state.lastAngle = std::atan2(dy, dx);

                state.angleTraversed = 0.0;
                state.secondRevolutionAngleTraversed = 0.0;

                // 不再使用 param1 作为当前半径
            }

            // --- 实时几何计算 ---
            const double dx = cur_pos.x - state.centerPosition.x;
            const double dy = cur_pos.y - state.centerPosition.y;
            const double current_radius = std::sqrt(dx * dx + dy * dy);
            const double angle = std::atan2(dy, dx);

            // 角度差（无跳变）
            double angle_diff = angle - state.lastAngle;
            while (angle_diff > M_PI)
                angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI)
                angle_diff += 2 * M_PI;
            state.lastAngle = angle;

            // --- 判断是否已进入小轨道（稳定环绕） ---
            state.inOrbit = (std::abs(current_radius - R_SMALL) < ORBIT_TOLERANCE);

            // --- 状态机控制 ---
            if (!state.hasCompletedFirstRevolution) {
                // ===== 第一阶段：小轨道盘旋 =====

                if (!state.inOrbit) {
                    state.angleTraversed = 0;
                } else {
                    // 累计角度（是否完成一圈小轨道）
                    state.angleTraversed += std::abs(angle_diff);
                }
                if (state.angleTraversed >= 2 * M_PI) {
                    state.hasCompletedFirstRevolution = true;
                }

                // 控制目标：保持在 R_SMALL 的小轨道上
                double radial_error = R_SMALL - current_radius;
                double radial_velocity = 0.0;

                if (std::abs(radial_error) > 1.0) {
                    // PD-like 控制进入小轨道
                    radial_velocity = 0.8 * radial_error;                                 // 比例控制
                    radial_velocity = std::clamp(radial_velocity, -speed / 2, speed / 2); // 限制速度
                }

                // 目标切向速度
                double target_speed = omega * R_SMALL;
                speed = std::min(target_speed, params.MAX_LEVELFLY_SPEED);

                // 合成方向
                double dir_radial = angle;
                double dir_tangent = angle + M_PI_2;

                double vx = radial_velocity * std::cos(dir_radial) + speed * std::cos(dir_tangent);
                double vy = radial_velocity * std::sin(dir_radial) + speed * std::sin(dir_tangent);
                direction = std::atan2(vy, vx);
                std::cout << radial_velocity << " " << direction << endl;

                // 前馈加速对齐
                double current_speed_mag =
                    std::sqrt(hull.velocity.x * hull.velocity.x + hull.velocity.y * hull.velocity.y);
                double speed_ratio = current_speed_mag / (speed + 1e-3);
                if (speed_ratio < 1.0) {
                    direction += (1.0 - speed_ratio) * 0.15;
                }
            } else {
                // ===== 第二阶段：扩展到 finalRadius 并飞外圈一圈 =====

                // 扩展径向速度（Sigmoid）
                double radial_velocity = 0.0;
                double radial_error = finalRadius - current_radius;

                if (current_radius < finalRadius && radial_error > 0.01) {
                    double e_norm = radial_error / finalRadius;
                    double sigmoid = 1.0 / (1.0 + std::exp(-6.0 * (e_norm - 0.25)));
                    radial_velocity = EXPAND_RATE * sigmoid;
                }

                // 外圈切向速度
                double target_speed = omega * current_radius;
                speed = std::min(target_speed, params.MAX_LEVELFLY_SPEED);

                // 合成方向
                double dir_radial = angle;
                double dir_tangent = angle + M_PI_2;

                double vx = radial_velocity * std::cos(dir_radial) + speed * std::cos(dir_tangent);
                double vy = radial_velocity * std::sin(dir_radial) + speed * std::sin(dir_tangent);
                direction = std::atan2(vy, vx);

                // 累计外圈角度（仅在接近 finalRadius 时）
                if (!state.hasCompletedSecondRevolution && current_radius >= finalRadius * 0.98) {
                    state.secondRevolutionAngleTraversed += std::abs(angle_diff);
                    if (state.secondRevolutionAngleTraversed >= 2 * M_PI) {
                        state.hasCompletedSecondRevolution = true;
                    }
                }
            }

            height = -cur_pos.z;
        }
    }
    if (flyflag && (full_charged || param.BATTERY == param.MAX_FLY_TIME)) {
        if ((c.getSpecificSingleton<Coordinate>().value().position - ReleasePosition).norm() >=
            param.MAX_CONTROL_RANGE) {
            c.getSpecificSingleton<Hull>().value().out_of_range = true;
            speed = 0;
        }
        QuadrotorMoveSystem::tickspecific(dt, c.getSpecificSingleton<Coordinate>().value(),
                                          c.getSpecificSingleton<Hull>().value(), direction, speed, height, param);
        full_charged = true;
    } else {
        param.BATTERY += dt * param.CHARGING_EFFICIENTY;
        if (param.BATTERY >= param.MAX_FLY_TIME) {
            full_charged = true;
        }
        param.BATTERY = param.BATTERY > param.MAX_FLY_TIME ? param.MAX_FLY_TIME : param.BATTERY;
        uavmodel::VID tmp = -1;
        for (auto& [k, v] : memberscan) {
            if (/*get<1>(v).baseInfo.type == uavmodel::BaseInfo::ENTITY_TYPE::SUPPORTCAR &&*/
                get<1>(v).baseInfo.side == c.getSpecificSingleton<SID>().value() &&
                c.getSpecificSingleton<PLATOONID>().value() == get<1>(v).baseInfo.platoonid) {
                tmp = k;
                break;
            }
        }
        if (tmp != -1) {
            c.getSpecificSingleton<Coordinate>().value().position = get<1>(memberscan[tmp]).position;
            c.getSpecificSingleton<Hull>().value().velocity = get<1>(memberscan[tmp]).velocity;
        }
    }
}
} // namespace uavmodel