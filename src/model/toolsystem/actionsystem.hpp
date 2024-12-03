#pragma once

#include "../tools/datastructure.hpp"
#include "../framework/system.hpp"

namespace uavmodel{

class ActionSystem : public System{
public:
    ActionSystem() = default;
    virtual void tick(double dt, Components& c) override {};
    virtual ~ActionSystem() = default;
};

}