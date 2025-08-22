#pragma once

#include "../tools/datastructure.hpp"

namespace uavmodel {

class Communication {
  public:
    virtual bool sendMessage(const Vector3& self, const Vector3& target) = 0;
    virtual double getcommPrPn(const Vector3& self, const Vector3& target, CommunicationData commdata) = 0;
    virtual ~Communication() = default;
};

} // namespace uavmodel