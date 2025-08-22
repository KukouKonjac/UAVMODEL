#pragma once

#include "sensor.hpp"
#include "radar/myradar.h"
#include "../framework/pattern.hpp"

namespace uavmodel{

struct SensorFactory : public Factory<SensorFactory, Sensor>{
    inline static LUT look_up_table{
        {"radar", std::make_shared<MyRadar>()},
        {"tanksensor",std::make_shared<TankSensor>()},
        {"scopesensor",std::make_shared<ScopeSensor>()},
        {"photoelectricitysensor", std::make_shared<photoelectricitySensor>()},
        {"supportvehicleradar", std::make_shared<SupportVehicleRadar>()},
        {"unmannedvehiclesensor", std::make_shared<UnmannedVehicleSensor>()},
        {"uavsensor", std::make_shared<UAVSensor>()},
        {"uavlasersensor", std::make_shared<UAVLaserSensor>()}
    };
};

};