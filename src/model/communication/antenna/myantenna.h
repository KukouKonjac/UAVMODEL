#pragma once

#include "../communication_d.hpp"

namespace uavmodel {

class MyAntenna : public Communication {
  public:
    MyAntenna() = default;
    virtual bool sendMessage(const Vector3& self, const Vector3& target) override;
    virtual double getcommPrPn(const Vector3& self, const Vector3& target, CommunicationData commdata) override;
    virtual ~MyAntenna() = default;
};

} // namespace uavmodel