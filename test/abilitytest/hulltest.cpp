#include "Windows.h"
#include <iostream>
#include <tuple>

#include "../basetest.h"
#define M_PI 3.14159265358979323846
using namespace std;

int main() {
    // 初始化无人机模型
    SetConsoleOutputCP(CP_UTF8);
    uavmodel::UavModel model;
    buildBaseModel("D:\\cqmodel\\rule_framework\\car.xml", model);

    // 获取命令缓冲区
    auto& buffer = model.components.getSpecificSingleton<uavmodel::CommandBuffer>().value();

    // 获取并初始化环绕状态
    auto& surroundState = model.components.getSpecificSingleton<uavmodel::SurroundState>().value();
    auto& motionParam = model.components.getSpecificSingleton<uavmodel::QuadrotorMotionParamList>().value();

    // 记录最大爬升高度
    double max_climb_height = -10;
    double max_levelfly_speed = 0;

    // 获取最大飞行时间
    double max_fly_time =
        model.components.getSpecificSingleton<uavmodel::QuadrotorMotionParamList>().value().MAX_FLY_TIME;

    // 写入文件
    std::vector<std::pair<double, double>> trajectory;
    const std::string base_path = "D:/GitHubProject/UAVMODEL/test/abilitytest/pythonProject/";

    bool Out_of_Range_Mark = false;
    int testmode = 5;
    double flag = false;
    if (testmode == 0) {
        for (int i = 0; i < 6000; i++) {
            auto& coordinate = model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position;
            double current_height = -coordinate.z;
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(8000, true)));
            max_climb_height = max(max_climb_height, current_height);
            // 输出：当前电量和当前高度
            cout << " 爬升高度:  " << current_height << "m" << endl;
            trajectory.emplace_back(i, -coordinate.z);
            model.tick(0.05);
             if ( current_height <= max_climb_height) {
                 break;
             }
        }
        cout << "==================================================" << endl;
        cout << "无人机升限 = " << max_climb_height << "m" << endl;
        cout << "==================================================" << endl;
    } else if (testmode == 1) {
        motionParam.BATTERY = 0;
        double max_charge_time = 0;
        while (motionParam.BATTERY < motionParam.MAX_FLY_TIME) {
            cout << "正在充电，当前电量 = " << motionParam.BATTERY / motionParam.MAX_FLY_TIME *100 << "%" << endl;
            max_charge_time += 0.05;
            model.tick(0.05);
            if ((motionParam.BATTERY == motionParam.MAX_FLY_TIME)) {
                cout << "充电完成，总耗时 = " << max_charge_time / 60 << "min" << endl;
            }
        }
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(800, true)));
        auto& coordinate = model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position;
        // 模拟飞行过程，循环执行3000次tick
        for (int i = 1; i <= 360000; ++i) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(3), any(tuple<double, double>(200, 0)));
            auto& velocity = model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.x;
            // 输出信息
            if (max_levelfly_speed < velocity || !max_levelfly_speed) {
                cout << "已充满电   当前速度:  " << velocity  << " " <<max_levelfly_speed<< "km/h" << endl;
                max_levelfly_speed = max(max_levelfly_speed, velocity);
            } else {
                max_levelfly_speed = max(max_levelfly_speed, velocity);
                cout << "最大巡航速度： " << max_levelfly_speed << endl;
                break;
            }
            model.tick(0.05);
        }
    } else if (testmode == 2) {
        get<1>((*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemory>()))[3]) =
            // EntityInfo{.position = {1000, 0, 0},
            EntityInfo{.position = {100, 0, 0},
                       .velocity = {20, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::SUPPORTCAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};

        model.components.getSpecificSingleton<uavmodel::SID>() = 1;
        model.components.getSpecificSingleton<uavmodel::PLATOONID>() = 1;
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(0), any(tuple<double, double>(true, true)));
        model.tick(0.05);
        auto ReleasePosition = model.components.getSpecificSingleton<Coordinate>().value().position;
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(8000, true)));
        model.tick(0.05);
        auto& coordinate = model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position;
        // 模拟飞行过程，循环执行3000次tick
        for (int i = 1; i <= 360000; ++i) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(3), any(tuple<double, double>(200, 0)));
            auto& velocity = model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity;
            // 输出信息
            if (max_levelfly_speed < velocity.norm()) {
                cout << "飞行时间:  " << setw(15) << fixed << motionParam.MAX_FLY_TIME - motionParam.BATTERY << "min"
                     << "   距离UAV操控台:  " << (coordinate - ReleasePosition).norm() / 1000 << "km" << endl;
            }
            model.tick(0.05);
            if (model.components.getSpecificSingleton<Hull>().value().out_of_range && !Out_of_Range_Mark) {
                cout << "==================================================" << endl;
                cout << "无人机超出控制范围，控制范围 = " << (coordinate - ReleasePosition).norm() / 1000 << "km"
                     << endl;
                cout << "==================================================" << endl;
                Out_of_Range_Mark = true;
                /*break;*/
            }

            if (motionParam.BATTERY <= 0) {
                cout << "==================================================" << endl;
                cout << "到达最大飞行时间 = " << fixed << setprecision(2) << (max_fly_time - motionParam.BATTERY) / 60
                     << "min" << endl;
                cout << "==================================================" << endl;
                break;
            }
        }
    } else if (testmode == 3) {
        get<1>((*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemory>()))[1]) =
            // EntityInfo{.position = {1000, 0, 0},
            EntityInfo{.position = {100, 0, 0},
                       .velocity = {20, 0, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::SUPPORTCAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};

        model.components.getSpecificSingleton<uavmodel::SID>() = 1;
        model.components.getSpecificSingleton<uavmodel::PLATOONID>() = 1;
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(0), any(tuple<double, double>(true, true)));
        model.tick(0.05);
        for (int i = 0; i < 100; i++) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(200, true)));
            model.tick(0.05);
            get<1>((*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemory>()))[1]).position.x +=
                0.05 * 20;
            cout << "Support Car Speed = "
                 << model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() / 3.6 << "km/h"
                 << endl;
            if (model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position.z <= -10) {
                cout << "UAV升空成功，Support Car 速度 = "
                     << model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity.norm() / 3.6 << "km/h"
                     << endl;
                break;
            }
        }

    } else if (testmode == 4) {
        get<1>((*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemory>()))[2]) =
            // EntityInfo{.position = {1000, 0, 0},
            EntityInfo{.position = {100, 0, 0},
                       .velocity = {60, 10, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::SUPPORTCAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        model.components.getSpecificSingleton<Coordinate>().value().position = {0, 0, 0};
        model.components.getSpecificSingleton<uavmodel::SID>() = 1;
        model.components.getSpecificSingleton<uavmodel::PLATOONID>() = 1;
        auto& cur_pos = model.components.getSpecificSingleton<Coordinate>().value().position;
        auto& cur_vel = model.components.getSpecificSingleton<Hull>().value().velocity;
        auto& tar_pos = get<1>((*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemory>()))[2]).position;
        auto& tar_vel = get<1>((*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemory>()))[2]).velocity;
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(0), any(tuple<double, double>(true, true)));
        model.tick(0.05);
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(200, true)));
        while (-cur_pos.z < 199) {
            tar_pos.x += 0.05 * 60;
            tar_pos.y += 0.05 * 10;
            model.tick(0.05);
            trajectory.emplace_back(cur_pos.x, cur_pos.y);
        }
        for (int i = 0; i < 1000; i++) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(3), any(tuple<double, double>(200, -1)));
            model.tick(0.05);
            trajectory.emplace_back(cur_pos.x, cur_pos.y);
        }
        cout << "成功起飞， 战车速度为： " << tar_vel.norm() << endl;
        double time_take = 0;
        while ((cur_pos - tar_pos).norm() > 0 || (cur_vel - tar_vel).norm() > 0) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(5), any(tuple<double, double>(2, true)));
            tar_pos.x += 0.05 * 60;
            tar_pos.y += 0.05 * 10;
            cout << "UAV与ZY战车距离" << (cur_pos - tar_pos).norm() << "m, 降落所用时间 =" << time_take << endl;
            model.tick(0.05);
            time_take += 0.05;
            trajectory.emplace_back(cur_pos.x, cur_pos.y);
            
        }
        cout << "成功降落，战车速度为： " << tar_vel.norm() << endl;

    } else if (testmode == 5) {
        get<1>((*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemory>()))[1]) =
            // EntityInfo{.position = {1000, 0, 0},
            EntityInfo{.position = {100, 0, 0},
                       .velocity = {60, 10, 0},
                       .baseInfo = {BaseInfo::ENTITY_TYPE::SUPPORTCAR, 1, 1, DAMAGE_LEVEL::N, 0, 1, 0, 3000.0, 1}};
        model.components.getSpecificSingleton<Coordinate>().value().position = {0, 0, 0};
        model.components.getSpecificSingleton<uavmodel::SID>() = 1;
        model.components.getSpecificSingleton<uavmodel::PLATOONID>() = 1;
        auto& cur_pos = model.components.getSpecificSingleton<Coordinate>().value().position;
        auto& cur_vel = model.components.getSpecificSingleton<Hull>().value().velocity;
        auto& tar_pos = get<1>((*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemory>()))[1]).position;
        auto& tar_vel = get<1>((*(model.components.getSpecificSingleton<uavmodel::SystemScannedMemory>()))[1]).velocity;
        cur_vel = {180, 0, 0};
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(0), any(tuple<double, double>(true, true)));
        model.tick(0.05);
        buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(1), any(tuple<double, double>(200, true)));
        while (-cur_pos.z < 199) {
            tar_pos.x += 0.05 * 60;
            tar_pos.y += 0.05 * 10;
            model.tick(0.05);
            trajectory.emplace_back(cur_pos.x, cur_pos.y);
        }
        trajectory.emplace_back(cur_pos.x, cur_pos.y);
        // 模拟飞行过程，循环执行3000次tick
        for (int i = 1; i <= 10000; ++i) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(6), any(tuple<double, double>(5000, 0)));
            model.tick(0.05);
            auto& state = model.components.getSpecificSingleton<uavmodel::SurroundState>().value();
            trajectory.emplace_back(cur_pos.x, cur_pos.y);
            cout << "当前速度 = " << cur_vel.norm() << endl;
            if (cur_vel.norm() > 180) {
                break;
            }
            
        }
     
        double time_take = 0;
        while (cur_vel.x!=tar_vel.x) {
            buffer.emplace(static_cast<uavmodel::command::COMMAND_TYPE>(5), any(tuple<double, double>(1, true)));
            
            tar_pos.x += 0.05 * 60;
            tar_pos.y += 0.05 * 10;
            cout << "UAV与ZY战车距离" << (cur_pos - tar_pos).norm() << "m" << cur_pos << endl;
            model.tick(0.05);
            time_take += 0.05;
            trajectory.emplace_back(cur_pos.x, cur_pos.y);
            
        }
        cout << time_take << endl;
    }
    std::ofstream file(base_path + "trajectory.csv");
    if (file.is_open()) {
        file << "x,y\n"; // CSV 头
        for (const auto& point : trajectory) {
            file << point.first << "," << point.second << "\n";
        }
        file.close();
        std::cout << "✅ 轨迹已保存到: " << base_path << "trajectory.csv" << std::endl;
    } else {
        std::cerr << "❌ 无法创建 trajectory.csv！路径可能错误或无权限: " << base_path << std::endl;
    }

    // === 🟢 写入参考圆（理想轨道）===
    std::ofstream ref_file(base_path + "circle_ref.csv");
    if (ref_file.is_open()) {
        ref_file << "cx,cy\n";
        for (int i = 0; i <= 360; ++i) {
            double theta = i * M_PI / 180.0;
            double x = 3000.0 + 3000.0 * std::cos(theta);
            double y = 0.0 + 3000.0 * std::sin(theta);
            ref_file << x << "," << y << "\n";
        }
        ref_file.close();
        std::cout << "✅ 参考圆已保存到: " << base_path << "circle_ref.csv" << std::endl;
    } else {
        std::cerr << "❌ 无法创建 circle_ref.csv！路径可能错误或无权限: " << base_path << std::endl;
    }

    return 0;
}