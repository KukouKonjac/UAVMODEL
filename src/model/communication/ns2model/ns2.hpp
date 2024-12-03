#pragma once

#include "../communication.hpp"

namespace wsfplugin {

struct NS2Model : public uavmodel::Communication {
    double params;
    virtual bool sendMessage(const uavmodel::Vector3& self, const uavmodel::Vector3& target){
        // TODO:
        
    }
    virtual ~NS2Model() = default;
};

} // namespace wsfplugin
