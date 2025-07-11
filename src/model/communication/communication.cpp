#include "communication.h"
#include "../../src/model/communication/communicationfactory.hpp"
namespace uavmodel {
double CommunicationSystem::v_fre = 0.0;
void CommunicationSystem::tick(double dt, Components& c) {
    // 计算两两之间的通信结果并存储到CommunicaionMemory中;（能否通信，其它参数如误码率、时延等）
    // auto& commu = c.getSpecificSingleton<CommunicaionMemory>().value();
    // 遍历scannedmemory所有车辆，利用hx写的通信算法计算通信结果并存储到CommunicaionMemory中;VID为map键，与每个车的CommuState为值;
    // ！首先通过阵营号筛选出同一阵营的车辆

    // from hx 计算两两之间的通信结果并存储到CommunicaionMemory中;（能否通信，其它参数如误码率、时延等）
    auto comm = CommunicationFactory::getProduct("antenna");
    auto& commmemory = c.getSpecificSingleton<CommunicaionMemory>().value();
    auto& memit = c.getSpecificSingleton<uavmodel::ScannedMemory>();            // 上帝视角所有无偏差实体
    auto& memsysscan = c.getSpecificSingleton<uavmodel::SystemScannedMemory>(); // 最终态势融合结果存储
    CommuState commstate;
    commstate.time_delay = INFINITY;
    ObstacleInfo obstacletmp;
    auto calculateSNR = [&](const Vector3& pos1, const Vector3& pos2) -> double {
        double SNR = comm->getcommPrPn(pos1, pos2);
        // 计算绕射损耗
        auto obstacletmp = EnvironmentInfoAgent{}.analyzeObstaclesBetween(pos1, pos2);
        if (obstacletmp.maxHeight < 0) {
            bool fre_flag = Fresnel(pos1, pos2, obstacletmp.maxObstaclePoint, c_speed / ff, obstacletmp.obstacleWidth);
            if (fre_flag) {
                SNR += CalFre(v_fre);
            }
        }
        return SNR;
    };
    std::vector<VID> uavid;
    for (auto&& [vid, entityInfo] : c.getSpecificSingleton<ScannedMemory>().value()) {
        // 计算无人机中继节点id
        if (get<1>(entityInfo).baseInfo.type == BaseInfo::ENTITY_TYPE::UAV &&
            get<1>(entityInfo).baseInfo.side == c.getSpecificSingleton<SID>().value() && vid != c.getSpecificSingleton<VID>().value())
            uavid.push_back(vid);
    }
    //  1.遍历scannedmemory所有我方车辆，计算是否可以通信，以及误码率和时延，存储到本车的CommuState中,这里的存储信息为DDS直接输入，还未经sensor.tick处理为真实探测
    for (auto&& [vid, entityInfo] : c.getSpecificSingleton<ScannedMemory>().value()) {
        // 计算通信结果并存储

        uavmodel::Vector3 myPos = c.getSpecificSingleton<Coordinate>().value().position;
        uavmodel::Vector3 otherPos = get<1>(entityInfo).position;
        if (get<1>(entityInfo).baseInfo.side == c.getSpecificSingleton<SID>().value()) {
            commstate.success = true;
            commstate.time_delay = 0.0;
            commstate.rate_error = 0.0;
            // if (vid == c.getSpecificSingleton<VID>().value()) {
            // } else {
            //     double SNR = calculateSNR(myPos, otherPos);
            //     // 2.计算误码率0~1，判断是否成功通信
            //     commstate.rate_error = 0.5 * erfc(sqrt(SNR));
            //     commstate.success = (commstate.rate_error < 1e-5);
            //     if (commstate.success) {
            //         double distance = (myPos - otherPos).norm();
            //         commstate.time_delay = distance / c_speed;
            //     }
            //     // 3.如果不能直接通信，则遍历查找无人机中继节点,选择能通信的中继节点且综合时延最小的；
            //     if (commstate.success == false) {
            //         for (int i = 0; i < uavid.size(); i++) {
            //             uavmodel::Vector3 uavPos = get<1>((*memit)[uavid[i]]).position;
            //             /*计算节点能否分别和本车以及目标车通信*/
            //             double rate_error1 = 0.5 * erfc(sqrt(calculateSNR(myPos, uavPos)));
            //             double rate_error2 = 0.5 * erfc(sqrt(calculateSNR(otherPos, uavPos)));
            //             bool success1 = rate_error1 < 1e-5;
            //             bool success2 = rate_error2 < 1e-5;
            //             if (success1 && success2) {
            //                 // 时延融合，误码率融合
            //                 commstate.success = true;
            //                 double newdelay = (myPos - uavPos).norm() / c_speed + (otherPos - uavPos).norm() /
            //                 c_speed; if (newdelay < commstate.time_delay) {
            //                     commstate.time_delay = newdelay;
            //                     commstate.rate_error = 1 - (1 - rate_error1) * (1 - rate_error2);
            //                 }
            //             }
            //         }
            //     }
            // }
            commmemory.emplace(vid, commstate); // vid为通信目标车辆的VID，CommuState为与此目标的通信状态
            // 根据通信结果计算态势融合结果
            // Deal with Systemscannedmemoryget to Systemscannedmemory
            if (commstate.success == true) {
                auto& info = (*(c.getSpecificSingleton<uavmodel::SystemScannedMemoryget>()))[vid];
                // 该能通信友车扫描到的实体列表，！！"应该是"也包含自己探测到的信息，会优先使用自己探测到的信息，自己探测不到的使用时延最小的探测到的信息
                for (auto& infomem : info) {
                    uavmodel::VID IDmem = infomem.first;
                    if (((*(memsysscan)).find(IDmem) != (*(memsysscan)).end()) &&
                        commstate.time_delay < get<0>((*(memsysscan))[IDmem])) { // 如果已有，则对比时延，选最小的
                        get<1>((*(memsysscan))[IDmem]) = get<1>(infomem.second);
                        get<0>((*(memsysscan))[IDmem]) = commstate.time_delay;         // 这里double存入通信时延
                    } else if ((*(memsysscan)).find(IDmem) == (*(memsysscan)).end()) { // 如果没有，直接存入
                        get<1>((*(memsysscan))[IDmem]) = get<1>(infomem.second);
                        get<0>((*(memsysscan))[IDmem]) = commstate.time_delay; // 这里double存入通信时延
                    }
                }
            }
        }
    }
    // TODO: 1.目前的中继通过找无人机一层中继实现；2.态势融合直接使用时延最小的，没有考虑误码或者各传感器的探测精度；
}
// 计算绕射常数
} // namespace uavmodel