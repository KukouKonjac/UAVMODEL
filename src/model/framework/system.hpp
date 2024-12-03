#pragma once

#include <string>
#include "../tools/datastructure.hpp"

namespace uavmodel{

struct SystemRestriction{
    std::vector<std::string> thisBefore;
    std::vector<std::string> thisAfter;
};

class System{
public:
    //! @param dt frame time, second
    virtual void tick(double dt, Components& c)=0;
    virtual ~System() = default;
};

};