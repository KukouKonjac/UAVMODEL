#include <iostream>
#include <tuple>

#include "../../src/model/sensors/radar/myradar.h"
#include "../basetest.h"

std::ostream& operator<<(std::ostream& o, const EntityInfo& e) {
    o << "{position: " << e.position << ", velocity: " << e.velocity << "}";
    return o;
}

int main() {
    using namespace std;
    // 测试模式，0为侦察高度（范围），1为识别目标距离或激光测距范围，2为目标定位精度或激光测距精度
    int testmode = 0;
    uavmodel::UavModel model;
    buildBaseModel("D:\\cqmodel\\rule_framework\\car.xml", model);
    // buildBaseModel("D:\\zgy2025\\uavtestxml\\uavsensor.xml", model);
    auto& buffer = model.components.getSpecificSingleton<uavmodel::CommandBuffer>().value();

    // testmode=0    侦察高度
    if (testmode == 0) {
        double min_height = 0;
        double max_height = 0;
        double lastdetectresult = -1;
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(2000, true)));
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
            // EntityInfo{.position = {1000, 0, 0},
            EntityInfo{.position = {100, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        for (int i = 0; i < 2000; ++i) {

            auto& tmp = model.components.getSpecificSingleton<uavmodel::ScannedMemory>().value()[1];
            if (lastdetectresult != 0 && get<0>(tmp) == 0 && lastdetectresult != -1) {
                min_height = -model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position.z;
            }
            if (lastdetectresult == 0 && get<0>(tmp) != 0 && lastdetectresult != -1) {
                max_height = -model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position.z;
            }
            lastdetectresult = get<0>(tmp);
            if (get<0>(tmp) != 0) {
                std::cout << "超出侦察高度范围 = "
                          << -model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position.z << " "
                          << "m" << endl;
            } else {
                std::cout << "进入侦察高度范围 = "
                          << -model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position.z << " "
                          << "m" << endl;
            }
            model.tick(0.05);
            get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
                EntityInfo{.position = {100, 0, 0},
                           .velocity = {0, 0, 0},
                           .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        }
        cout << "==================================================" << endl;
        cout << "最小侦察高度 = " << min_height << "m" << endl;
        cout << "==================================================" << endl;
        cout << "==================================================" << endl;
        cout << "最大侦察高度 = " << max_height << "m" << endl;
        cout << "==================================================" << endl;
    }

    // testmode=1   识别目标距离
    if (testmode == 1) {
        model.components.getSpecificSingleton<Coordinate>().value().position.z = -800;
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(800, true)));
        model.tick(0.05);
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(3), any(tuple<double, double>(20, 0)));
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
            EntityInfo{.position = {5000, 0, -800},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::UAV, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        double min_range = 0;
        double max_range = 0;
        double lastdetectresult = -1;

        while ((model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]).position)
                   .norm() >= 10) {
            model.tick(0.05);
            auto& tmp = model.components.getSpecificSingleton<uavmodel::ScannedMemory>().value()[1];
            if (lastdetectresult != 0 && get<0>(tmp) == 0 && lastdetectresult != -1) {
                max_range = (model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                             get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]).position)
                                .norm();
            }
            if (lastdetectresult == 0 && get<0>(tmp) != 0 && lastdetectresult != -1) {
                min_range = (model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                             get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]).position)
                                .norm();
            }
            lastdetectresult = get<0>(tmp);
            if (get<0>(tmp) != 0) {
                std::cout << "超出侦察范围 = "
                          << (model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                              get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]).position)
                                 .norm()
                          << "m " << endl;
            } else {
                std::cout << "进入侦察范围 = "
                          << (model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                              get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]).position)
                                 .norm()
                          << "m " << endl;
            }
            get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
                EntityInfo{.position = {5000, 0, -800},
                           .velocity = {0, 0, 0},
                           .baseInfo = {BaseInfo::ENTITY_TYPE::UAV, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        }
        if (min_range == 0) {
            cout << "==================================================" << endl;
            cout << "识别目标距离 = " << max_range << "m" << endl;
            cout << "==================================================" << endl;
        } else {
            cout << "==================================================" << endl;
            cout << "激光测距范围下界 = " << min_range << "m" << endl;
            cout << "==================================================" << endl;
            cout << "==================================================" << endl;
            cout << "激光测距范围上界 = " << max_range << "m" << endl;
            cout << "==================================================" << endl;
        }
    }

    // testmode=2    目标定位精度
    else if (testmode == 2) {
        double detectrate = 0;
        double jd = 0;
        model.components.getSpecificSingleton<Coordinate>().value().position.z = -800;
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
            // EntityInfo{.position = {1000, 0, 0},
            EntityInfo{.position = {1000, 0, -800},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::UAV, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(800, true)));
        for (int i = 0; i < 1000; ++i) {
            model.tick(0.05);
            auto& tmp = model.components.getSpecificSingleton<uavmodel::ScannedMemory>().value()[1];
            if (get<0>(tmp) == 0) {
                detectrate += 1; // cout << "scanned list: " << get<1>(tmp) << endl;
                jd += sqrt((get<1>(tmp).position.x - 1000) * (get<1>(tmp).position.x - 1000) +
                           get<1>(tmp).position.y * get<1>(tmp).position.y);
            } else {
                ; // cout << "scanned list: {}" << endl;
            }
            get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
                EntityInfo{.position = {1000, 0, -800},
                           .velocity = {0, 0, 0},
                           .baseInfo = {BaseInfo::ENTITY_TYPE::UAV, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        }
        std::cout /*<< "探测距离："
                            << (get<1>(tmp).position -
                            model.components.getSpecificSingleton<Coordinate>().value().position).norm()*/
            << "目标定位精度：" << jd / detectrate << "m" << endl;
    }
    return 0;
}