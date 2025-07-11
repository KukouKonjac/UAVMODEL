#pragma once

#include "../environment.h"
#include "../framework/system.hpp"
#include "../tools/datastructure.hpp"

namespace uavmodel {
class CommunicationSystem : public System {
  public:
    // virtual bool sendMessage(const Vector3& self, const Vector3& target)=0;
    CommunicationSystem() = default;
    virtual void tick(double dt, Components& c) override;
    virtual ~CommunicationSystem() override = default;
    static double v_fre;
    static double CalDistancexyz(const Vector3& pos1, const Vector3& pos2) {
        double dx = pos2.x - pos1.x;
        double dy = pos2.y - pos1.y;
        double dz = pos2.z - pos1.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    static double CalDistancexy(const Vector3& pos1, const Vector3& pos2) {
        double dx = pos2.x - pos1.x;
        double dy = pos2.y - pos1.y;
        return std::sqrt(dx * dx + dy * dy);
    };

    static bool Fresnel(const Vector3& pos1, const Vector3& pos2, const Vector3& obs1, double lambda, double W_obs) {
        double d1 = CalDistancexy(pos1, obs1);
        double d2 = CalDistancexy(pos2, obs1);
        double H = obs1.z; // 等效山体高度

        Vector3 center = (pos1 + pos2) / 2;
        double r = sqrt(lambda * (d1 * d2) / (d1 + d2)); // 计算菲涅尔第一区域

        double S_fre = 3.1415926 * r * r;
        CalDistancexyz(center, obs1);
        double S_obs = 0.5 * H * W_obs;
        if (S_obs > 0.55 * S_fre) {
            v_fre = -H * sqrt(2 / (lambda) * (1 / d1 + 1 / d2));
            return true;
        } else {
            return false;
        }
    };
    // 绕射损耗
    static double CalFre(double v1) {
        if (v1 >= 1) {
            return 0;
        } else if (v1 < 1 && v1 >= 0) {
            return 20 * log10(0.5 + 0.62 * v1);
        } else if (v1 < 0 && v1 >= -1) {
            return 20 * log10(0.5 * exp(0.45 * v1));
        } else if (v1 < -1 && v1 >= -2.4) {
            return 20 * log10(0.4 - sqrt(0.1184 - (0.1 * v1 + 0.38) * (0.1 * v1 + 0.38)));
        } else if (v1 <= -2.4)
            return 20 * log10(-0.225 / v1);
    };

  private:
    double c_speed = 3e8;
    double ff = 0.1e9;
};

} // namespace uavmodel