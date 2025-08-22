#pragma once

#include "../environment.h"
#include "../framework/system.hpp"
#include "../tools/datastructure.hpp"

// 标准坐标系：北东地与前右下

namespace uavmodel {

class HullSystem : public System {
  public:
    HullSystem() = default;
    virtual void tick(double dt, Components& c) override;
    virtual ~HullSystem() = default;
    double height = 0;
    double speed = 0;
    double direction = 0;
    bool flyflag = 0;
    Vector3 ReleasePosition = {0, 0, 0};
    bool full_charged = false;
};

} // namespace uavmodel