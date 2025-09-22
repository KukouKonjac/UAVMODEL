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
        if (get<1>(v).baseInfo.type == uavmodel::BaseInfo::ENTITY_TYPE::SUPPORTCAR &&
            get<1>(v).baseInfo.side == c.getSpecificSingleton<SID>().value() &&
            c.getSpecificSingleton<PLATOONID>().value() == get<1>(v).baseInfo.platoonid) {
            tmp = k;
            break;
        }
    }
    if (flyflag == false) {
        if (tmp == -1) {
            ReleasePosition = c.getSpecificSingleton<Coordinate>().value().position;
        } else {
            ReleasePosition = get<1>(memberscan[tmp]).position;
        }
    } else {
        if (tmp != -1) {
            ReleasePosition = get<1>(memberscan[tmp]).position;
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
            speed =
                std::sqrt(c.getSpecificSingleton<Hull>().value().velocity.x * c.getSpecificSingleton<Hull>().value().velocity.x +
                c.getSpecificSingleton<Hull>().value().velocity.y * c.getSpecificSingleton<Hull>().value().velocity.y);
            height = param1;
            flyflag = true;
        } else if (k == COMMAND_TYPE::LEVELFLIGHT) {
            speed = param1;
            direction = param2;
        } else if (k == COMMAND_TYPE::HOVER) {
            speed = 0;
        } else if (k == COMMAND_TYPE::BACK || k == COMMAND_TYPE::FOLLOWCAR) {
            auto& tar_pos = std::get<1>(c.getSpecificSingleton<SystemScannedMemory>().value()[param1]).position;
            auto& tar_vel = std::get<1>(c.getSpecificSingleton<SystemScannedMemory>().value()[param1]).velocity;

            double target_x = tar_pos.x;
            double target_y = tar_pos.y;
            double target_z = -tar_pos.z; // 无人机目标高度
            double offset = 50;
            if (std::get<1>(c.getSpecificSingleton<SystemScannedMemory>().value()[param1]).baseInfo.side !=
                    c.getSpecificSingleton<SID>().value() &&
                flyflag) {
                target_x -= offset;
                target_y -= offset;

            }

            const auto& cur_pos = c.getSpecificSingleton<Coordinate>().value().position;
            const auto& cur_vel = c.getSpecificSingleton<Hull>().value().velocity;
            double dx = target_x - cur_pos.x;
            double dy = target_y - cur_pos.y;
            double dz = target_z + cur_pos.z;

            double horizontal_dist = std::sqrt(dx * dx + dy * dy);

            const double Kp = 0.5;
            const double Ki = 0.0;
            const double Kd = 0.8;
            static double integral_x = 0.0;
            static double prev_error_x = 0.0;
            static double integral_y = 0.0;
            static double prev_error_y = 0.0;


            // =============================
            // X 方向 PID
            // =============================
            integral_x += dx * dt;
            integral_x = std::clamp(integral_x, -1.0, 1.0); // 积分限幅
            double derivative_x = (dx - prev_error_x) / dt;
            double desired_vx = tar_vel.x + Kp * dx + Ki * integral_x + Kd * derivative_x;
            prev_error_x = dx;

            // =============================
            // Y 方向 PID
            // =============================
            integral_y += dy * dt;
            integral_y = std::clamp(integral_y, -1.0, 1.0); // 积分限幅
            double derivative_y = (dy - prev_error_y) / dt;
            double desired_vy = tar_vel.y + Kp * dy + Ki * integral_y + Kd * derivative_y;
            prev_error_y = dy;

            // 限制最大速度
            double max_speed = optParam.value().MAX_LEVELFLY_SPEED;
            double desired_speed = std::sqrt(desired_vx * desired_vx + desired_vy * desired_vy);
            if (desired_speed > max_speed) {
                double scale = max_speed / desired_speed;
                desired_vx *= scale;
                desired_vy *= scale;
            }
            // 输出给 tickspecific
            if (flyflag) {
                speed = desired_speed;                          // 用于控制油门
                direction = std::atan2(desired_vy, desired_vx); // 期望速度方向
            }
            

            if (k == COMMAND_TYPE::BACK) {
                height = target_z; // 保持目标高度
            } 
            if ((tar_pos - cur_pos).norm() < 3 && (cur_vel - tar_vel).norm() < 3) {
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
            const double omega = 0.15;                          // 角速度 (rad/s)

            const double ORBIT_TOLERANCE = 30.0; // 允许误差：±50m

            // --- 第一次初始化：计算圆心，设置状态 ---
            if (!state.isSurrounding) {
                state.isSurrounding = true;

                // 圆心：仍使用 param1 和 param2（固定点）
                state.centerPosition.x = cur_pos.x + param1 * std::cos(param2);
                state.centerPosition.y = cur_pos.y + param1 * std::sin(param2);
                state.centerPosition.z = cur_pos.z;

                // 初始化角度（从当前位置指向圆心）
                const double dx = cur_pos.x - state.centerPosition.x;
                const double dy = cur_pos.y - state.centerPosition.y;
                state.lastAngle = std::atan2(dy, dx);

                state.angleTraversed = 0.0;
                state.secondRevolutionAngleTraversed = 0.0;
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
                    //radial_velocity = std::clamp(radial_velocity, -speed, speed); // 限制速度
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

                // 前馈加速对齐
                double current_speed_mag =
                    std::sqrt(hull.velocity.x * hull.velocity.x + hull.velocity.y * hull.velocity.y);
                double speed_ratio = current_speed_mag / (speed + 1e-3);
                if (speed_ratio < 1.0) {
                    direction += (1.0 - speed_ratio) * 0.15;
                }
            } else {
                // ===== 第二阶段：持续螺旋扩展，每圈外扩 500m =====

                const double R_INCREMENT = 500.0;            // 每圈外扩 500 米
                const double TARGET_RADIUS_TOLERANCE = 30.0; // 接近目标半径的判定阈值

                // 动态更新目标半径：基于已环绕的圈数
                double target_radius = R_SMALL + R_INCREMENT * state.completedCircles;

                // 判断是否已经进入当前圈的目标轨道（用于角度累计）
                bool inTargetOrbit = (std::abs(current_radius - target_radius) < TARGET_RADIUS_TOLERANCE);

                // 扩展径向速度：向当前目标半径靠近
                double radial_velocity = 0.0;
                double radial_error = target_radius - current_radius;

                if (std::abs(radial_error) > 1.0) {
                    // 使用 PD 控制或限幅比例控制
                    double kp = 0.5;
                    double kd = 0.5;
                    double d_radial_error = radial_error - state.lastRadialError;
                    double dt = 1.0 / 50.0; // 假设 50Hz 控制频率，可根据实际调整
                    double derivative = d_radial_error / dt;

                    radial_velocity = kp * radial_error + kd * derivative;
                    radial_velocity = std::clamp(radial_velocity, -speed, speed);
                }

                // 切向速度：保持恒定角速度 omega
                double target_speed = omega * current_radius;
                speed = std::min(target_speed, params.MAX_LEVELFLY_SPEED);

                // 合成方向
                double dir_radial = angle;           // 径向朝外
                double dir_tangent = angle + M_PI_2; // 切向（逆时针）

                double vx = radial_velocity * std::cos(dir_radial) + speed * std::cos(dir_tangent);
                double vy = radial_velocity * std::sin(dir_radial) + speed * std::sin(dir_tangent);
                direction = std::atan2(vy, vx);

                // 累计角度（仅在接近当前目标半径时才累计，防止误判）
                if (inTargetOrbit) {
                    state.secondRevolutionAngleTraversed += std::abs(angle_diff);
                }

                // 检查是否完成一圈（360°）
                if (state.secondRevolutionAngleTraversed >= 2 * M_PI) {
                    state.completedCircles++;                   // 圈数 +1
                    state.secondRevolutionAngleTraversed = 0.0; // 重置角度累计
                    // 下一圈的目标半径自动增加 R_INCREMENT
                }

                // 更新 lastRadialError 用于微分项
                state.lastRadialError = radial_error;
            }

        }
    }
    if (flyflag && (full_charged || param.BATTERY == param.MAX_FLY_TIME)) {
        if ((c.getSpecificSingleton<Coordinate>().value().position - ReleasePosition).norm() >=
            param.MAX_CONTROL_RANGE) {
            c.getSpecificSingleton<Hull>().value().out_of_range = true;
            speed = 0;
        }
        else {
            c.getSpecificSingleton<Hull>().value().out_of_range = false;
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
        for (auto& [k, v] : memberscan) {
            if (get<1>(v).baseInfo.type == uavmodel::BaseInfo::ENTITY_TYPE::SUPPORTCAR &&
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