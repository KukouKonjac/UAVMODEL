#include <iostream>
#include <tuple>

#include "../basetest.h"
int main() {
    int testmode = 5;
    using namespace std;
    uavmodel::UavModel model;
    buildBaseModel("D:\\cqmodel\\v\\uav\\uav.xml", model);
    auto& buffer = model.components.getSpecificSingleton<uavmodel::CommandBuffer>().value();
    int flag = 0;
    //测试维修保障指标/最大速度/加速时间-------------------------------
    if (testmode == 1)
    {
        auto& damage = model.components.getSpecificSingleton<uavmodel::DamageModel>().value();
        damage.damageLevel = DAMAGE_LEVEL::K;
        double acc_time = 0;
        for (int i = 0; i < 200; ++i) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(2), any(tuple<double, double>(100, 0)));
            model.tick(0.1);
            if (damage.damageLevel == DAMAGE_LEVEL::N &&
                model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() == 0) {
                cout << "fix time: " << 0.1 * i << " current velocity: "
                     << model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() * 3.6
                     << "km/h" << endl;
            } else if (damage.damageLevel == DAMAGE_LEVEL::N) {
                cout << "fixed "
                     << " current velocity: "
                     << model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() * 3.6
                     << "km/h" << endl;
                acc_time += 0.1;
            } else
                cout << "fixing..."
                     << "current velocity: "
                     << model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() * 3.6
                     << "km/h" << endl;
            if (model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() * 3.6 >= 32 &&
                flag == 0) {
                cout << "加速时间： " << acc_time << "s" << endl;
                flag = 1;
            }
        }
    }
    else if(testmode == 2)////测试制动距离---------------------------------
    {
        for (int i = 0; i < 200; ++i) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(2), any(tuple<double, double>(100, 0)));
            model.tick(0.1);
        }
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(5), any(tuple<double, double>(0, 0)));
        model.tick(0.1);

        uavmodel::Vector3 pos_startstop;
        for (int i = 0; i < 200; ++i) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(5), any(tuple<double, double>(0, 0)));
            model.tick(0.1);
            cout << "stoping... "
                 << "current velocity: "
                 << model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() * 3.6 << "km/h"
                 << endl;
            if (model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() * 3.6 <= 32 &&
                flag == 1) {
                pos_startstop = model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position;
                flag = 2;
            }
            if (model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() == 0) {
                cout << "stop distance: "
                     << (model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                         pos_startstop)
                            .norm()
                     << "m" << endl;
                break;
            }
        }
    }
    else if(testmode == 3){////测试装甲防护指标----------------------------
        auto& protect = get<1>(*model.components.getNormal<uavmodel::ProtectionModel>().begin());
        cout << "armor front: " << protect.armor_front << "m" << endl
             << "armor back: " << protect.armor_back << "m" << endl
             << "armor side: " << protect.armor_side << "m" << endl
             << "armor top: " << protect.armor_top << "m" << endl
             << "armor bottom: " << protect.armor_bottom << "m" << endl;
    }
    else if(testmode == 4)// 测试最大行程
    {
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(2), any(tuple<double, double>(100, 0)));
        model.tick(0.1);

        uavmodel::Vector3 pos_startstop =
            model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position;
        for (int i = 0; i < 2000; ++i) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(2), any(tuple<double, double>(100, 0)));
            model.tick(100);
            cout << "stoping... "
                 << "current velocity: "
                 << model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() * 3.6 << "km/h"
                 << endl;
            if (model.components.getSpecificSingleton<uavmodel::WheelMotionParamList>().value().OIL_REMAIN - 0 <
                0.01) {
                cout << "最大行程: "
                     << (model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position -
                         pos_startstop)
                                .norm() /
                            1000
                     << "km" << endl;
                break;
            }
        }
    } 
    else if(testmode == 5) //测试无人机运动平台模型
    {
        /*buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(4), any(tuple<double, double>(20, PI / 4)));
        model.tick(0.01);
        for (int i = 0; i < 1000; ++i) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(4), any(tuple<double, double>(20, PI / 4)));
            model.tick(0.01);
            std::cout << "Position Error: " << uavmodel::Vector3{100, 100, 0} - model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position << std::endl;
            std::cout << "Velocity " << model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.x<< "m/s" << std::endl;
            std::cout << "height" << model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position.z << "m" << std::endl;
            std::cout << "time " << 0.01 * i << "s" << std::endl;
        }*/
        for (int i = 0; i < 1000; ++i) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(2), any(tuple<double, double>(20, PI / 4)));
            model.tick(0.01);
            std::cout << "Velocity " << model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.x
                      << "m/s" << std::endl;
            std::cout << "height" << model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position.z * -1 
                      << "m" << std::endl;
            std::cout << "time " << 0.01 * i << "s" << std::endl;
        }
    }
    return 0;
}