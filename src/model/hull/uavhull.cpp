#include <cmath>
#include <set>
#include <tuple>

#include "../tools/vector3.hpp"
#include "quadrotor/quadrotor.h"
#include "uavhull.h"
#include "wheel/wheel.h"

namespace {

using namespace uavmodel::command;

constexpr size_t validMovingCommandMask =
    size_t(1) << static_cast<int>(COMMAND_TYPE::FOLLOWCAR) | size_t(1) << static_cast<int>(COMMAND_TYPE::CLIMB) |
    size_t(1) << static_cast<int>(COMMAND_TYPE::DIVE) | size_t(1) << static_cast<int>(COMMAND_TYPE::LEVELFLIGHT) |
    size_t(1) << static_cast<int>(COMMAND_TYPE::HOVER) | size_t(1) << static_cast<int>(COMMAND_TYPE::BACK);

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
            speed = param1;
            direction = param2;
        } else if (k == COMMAND_TYPE::HOVER) {
            speed = 0;
        } else if (k == COMMAND_TYPE::BACK) {
            auto& tar = std::get<1>(c.getSpecificSingleton<SystemScannedMemory>().value()[param1]).position;
            if (abs(tar.z - coordinate.position.z) < 1) {
                flyflag = false;
                continue;
            }
            double dis = sqrt(pow((c.getSpecificSingleton<Coordinate>().value().position.x - tar.x), 2) + pow((c.getSpecificSingleton<Coordinate>().value().position.y - tar.y), 2));
            speed = dis > 200 ? /*20 : floor(dis / 20)*/ speed : floor(dis / 20);
            speed = speed < 0 ? 0 : speed;
            if (speed == 0) {
                height = -1 * tar.z;
            }
            direction = atan2(tar.y - c.getSpecificSingleton<Coordinate>().value().position.y,
                              tar.x - c.getSpecificSingleton<Coordinate>().value().position.x);
        }
    }
    if (flyflag) {
        QuadrotorMoveSystem::tickspecific(dt, c.getSpecificSingleton<Coordinate>().value(),
                                  c.getSpecificSingleton<Hull>().value(), direction, speed, height, param);
    } else {
        param.MAX_FLY_TIME += dt * 0.5;
        param.MAX_FLY_TIME = param.MAX_FLY_TIME > 2400 ? 2400 : param.MAX_FLY_TIME;
        uavmodel::VID tmp = -1;
        auto& sysscan = c.getSpecificSingleton<SystemScannedMemory>().value();
        for (auto& [k, v] : sysscan) {
            if (/*get<1>(v).baseInfo.type == uavmodel::BaseInfo::ENTITY_TYPE::SUPPORTCAR &&*/
                get<1>(v).baseInfo.side == c.getSpecificSingleton<SID>().value() &&
                c.getSpecificSingleton<PLATOONID>().value() == get<1>(v).baseInfo.platoonid) {
                tmp = k;
                break;
            }
        }
        if (tmp != -1) {
            c.getSpecificSingleton<Coordinate>().value().position = get<1>(sysscan[tmp]).position;
            c.getSpecificSingleton<Hull>().value().velocity = get<1>(sysscan[tmp]).velocity;
        }
    }
}
} // namespace uavmodel