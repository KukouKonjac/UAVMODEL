#pragma once

#include "../communication.hpp"

namespace uavmodel{

class MyAntenna : public Communication{
public:
    MyAntenna() = default;
    virtual bool sendMessage(const Vector3& self, const Vector3& target) override;
    virtual ~MyAntenna() = default;
};

}