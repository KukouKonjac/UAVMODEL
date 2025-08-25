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
    int testmode = 1;
    if (testmode == 0) {
        model.components.getSpecificSingleton<uavmodel::SID>() = 1;
        auto& buffer = model.components.getSpecificSingleton<uavmodel::CommandBuffer>().value();
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(0, 0)));
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(3), any(tuple<double, double>(200, 0)));
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
            // EntityInfo{.position = {1000, 0, 0},
            EntityInfo{.position = {5000, 0, 0},
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
                EntityInfo{.position = {5000, 0, 0},
                           .velocity = {0, 0, 0},
                           .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        }

    } else if (testmode == 1) {
        model.components.getSpecificSingleton<uavmodel::SID>() = 1;
        model.components.getSpecificSingleton<uavmodel::VID>() = 0;
        auto& buffer = model.components.getSpecificSingleton<uavmodel::CommandBuffer>().value();
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(0, true)));
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(3), any(tuple<double, double>(20, 0)));
        // 初始化距离参数
        double launch_time = 0.5;
        double receive_time = 0.5;
        double transmit_time_4km = 0.000013;
        double transmit_time_5km = 0.000016;
        double transmit_time_6km = 0.000021;
        double transfer_time = 0.3;

        // 0车的ScannedMemory通信车辆位置
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[1]) =
            EntityInfo{.position = {5000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[2]) =
            EntityInfo{.position = {10000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[4]) =
            EntityInfo{.position = {10000, 4000, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[3]) =
            EntityInfo{.position = {15000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[5]) =
            EntityInfo{.position = {20000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};

        // 1车内部信息（与0,2,3）
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[1][2] = {
            launch_time + receive_time + transmit_time_5km,
            EntityInfo{.position = {10000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[2][1] = {
            launch_time + receive_time + transmit_time_5km,
            EntityInfo{.position = {5000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[1][4] = {
            launch_time + receive_time + transmit_time_6km,
            EntityInfo{.position = {10000, 4000, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[4][1] = {
            launch_time + receive_time + transmit_time_6km,
            EntityInfo{.position = {5000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        // 2车内部信息（与0,1,3，4）
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[2][4] = {
            launch_time + receive_time + transmit_time_4km,
            EntityInfo{.position = {10000, 4000, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[4][2] = {
            launch_time + receive_time + transmit_time_4km,
            EntityInfo{.position = {10000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[2][3] = {
            launch_time + receive_time + transmit_time_5km,
            EntityInfo{.position = {15000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[3][2] = {
            launch_time + receive_time + transmit_time_5km,
            EntityInfo{.position = {10000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        // 3车内部信息（与1,2,4）
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[4][3] = {
            launch_time + receive_time + transmit_time_6km,
            EntityInfo{.position = {15000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[3][4] = {
            launch_time + receive_time + transmit_time_6km,
            EntityInfo{.position = {10000, 4000, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        // 4车内部信息（与2,3,5）
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[3][5] = {
            launch_time + receive_time + transmit_time_5km,
            EntityInfo{.position = {20000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};
        (*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[5][3] = {
            launch_time + receive_time + transmit_time_5km,
            EntityInfo{.position = {15000, 0, 0},
                       .velocity = {0, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::CAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}}};

        double min_height = 0;
        double max_hetght = 0;
        model.tick(0.05);
        auto& optresult = model.components.getSpecificSingleton<uavmodel::OptCommResult>().value();
        auto& optcomm = model.components.getSpecificSingleton<uavmodel::OptCommMemory>().value();

        std::cout << "\n=== print communication link ===" << std::endl; // 计算最短时延
        for (const auto& [src, dstMap] : optresult) {                   // 外层循环：源节点 -> 目的节点映射表
            for (const auto& [dst, dr] : dstMap) { // 内层循环：目的节点 -> 路径信息（修复变量名冲突）
                double timedelay = get<0>(dr);
                bool reachable = get<1>(dr);
                auto path = get<2>(dr);

                if (src == dst || !reachable)
                    continue; // 跳过自身节点和不可达节点
                if (path.size() == 5) {
                    // 四跳通信（4跳：5个节点）
                    std::cout << "4-hop link: ";
                    // 打印完整路径
                    for (size_t i = 0; i < path.size(); ++i) {
                        std::cout << path[i];
                        if (i != path.size() - 1) {
                            std::cout << " -> ";
                        }
                    }
                    std::cout << " | [4-hop] total time delay: " << timedelay << "ms" << std::endl << std::endl;
                    std::cout << "single-hop communication link segment used in the communication process: "
                              << std::endl;
                    // 打印每段通信链路（共4段）
                    for (size_t i = 0; i < path.size() - 1; ++i) {

                        std::cout << path[i] << " -> " << path[i + 1];
                        std::cout << " | [1-hop] time delay: " << get<0>(optcomm[path[i]][path[i + 1]]).time_delay
                                  << "ms" << std::endl;
                    }
                    std::cout << std::endl;
                }
            }
        }
        std::cout << "===========================\n" << std::endl;
    }
    else if (testmode == 2) {
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