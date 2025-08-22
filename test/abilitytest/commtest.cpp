#include <iostream>
#include <string_view>
#include <tuple>

#include "../../src/model/communication/antenna/extern/Comm.h"
#include "../../src/model/communication/communicationfactory.hpp"
#include "../../src/model/sensors/radar/myradar.h"
#include "../basetest.h"

std::ostream& operator<<(std::ostream& o, const EntityInfo& e) {
    o << "{position: " << e.position << ", velocity: " << e.velocity << "}";
    return o;
}

int main() {
    using namespace std;
    uavmodel::UavModel model;
    buildBaseModel("D:\\cqmodel\\rule_framework\\car.xml", model);
    int testmode = 0;
    if (testmode == 0) {
        model.components.getSpecificSingleton<uavmodel::SID>() = 1;
        auto& buffer = model.components.getSpecificSingleton<uavmodel::CommandBuffer>().value();
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(0, 0)));
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(3), any(tuple<double, double>(200, 0)));
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
            // EntityInfo{.position = {1000, 0, 0},
            EntityInfo{.position = {12000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        double min_height = 0;
        double max_hetght = 0;
        while ((model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]).position)
                   .norm() >= 10) {
            model.tick(0.05);
            auto& tmp = model.components.getSpecificSingleton<uavmodel::CommunicaionMemory>().value()[1];
            if (tmp.success) {
                std::cout << "Within the comm range = "
                          << (model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                              get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]).position)
                                 .norm()
                          << "  Time Delay = " << tmp.time_delay << endl;
                break;
            } else {
                std::cout << "Not within the detection range = "
                          << (model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                              get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]).position)
                                 .norm()
                          << "  Time Delay = " << tmp.time_delay << endl;
            }
            get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
                EntityInfo{.position = {12000, 0, 0},
                           .velocity = {0, 0, 0},
                           .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        }

    } else if (testmode == 1) {
        using namespace externModel::comm;
        Comm comm;
        comm.Init(10.0);
        comm.SetInput({0, 0, 0}, {1, 1, 1}, false, {0., 0., 0.});
        double hoped_distance;
        std::cin >> hoped_distance;
        double power_needed = comm.getRequiredTransmitPower(hoped_distance);
        cout << "hoped_distance is " << hoped_distance << "M" << '\t' << "need_tran_power " << power_needed << "W"
             << '\n'
             << endl;
    }

    return 0;
}