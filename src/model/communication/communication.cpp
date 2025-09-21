#include "communication.h"
#include "../../src/model/communication/communicationfactory.hpp"
#include <queue>
namespace uavmodel {
double CommunicationSystem::v_fre = 0.0;

struct DelayResult {
    double delay;          // 最短时延
    bool reachable;        // 是否可达
    std::vector<VID> path; // 完整路径（src -> ... -> dst）
};

struct IndexEntry {
    int tno;
    int rno;
    double time_delay;
    IndexEntry(int a, int b, double c) : tno(a), rno(b), time_delay(c) {}
};

class CommunicationAnalyzer {
  public:
    using VID = int;
    using AdjList = std::map<VID, std::vector<std::pair<VID, double>>>;
    AdjList adj;

    // 20250820 hx 中继节点的转发时延获取transfer_delay
    // 暂时为一个car.xml文件中trandelay，后续由话题发布加入optcommMemory
    void buildGraph(const OptCommMemory& commu) {
        adj.clear();
        for (const auto& [src, targets] : commu) {
            for (const auto& [dst, state_data] : targets) {
                const auto& [state] = state_data;
                if (state.success && src != dst) { // 排除自身到自身的边
                    adj[src].emplace_back(dst, state.time_delay);
                }
            }
        }
    }

    // 计算最短路径并返回包含路径的结果
    std::map<VID, DelayResult> calculateDelays(VID src, Components& c, CommunicationData& bestcomm) {
        std::map<VID, double> dist; // 最短时延
        std::map<VID, VID> prev;    // 前驱节点（用于回溯路径）
        std::priority_queue<std::pair<double, VID>, std::vector<std::pair<double, VID>>, std::greater<>> pq;
        auto& optresult = c.getSpecificSingleton<OptCommResult>().value();
        double transfer_delay = bestcomm.transdelay;
        // 初始化：所有节点时延设为无穷大，前驱设为-1（无效值）
        for (const auto& [node, _] : adj) {
            dist[node] = std::numeric_limits<double>::infinity();
            prev[node] = -1; // 标记无前驱
        }
        dist[src] = 0.0;
        pq.emplace(0.0, src);

        // Dijkstra算法核心：更新最短时延和前驱节点
        // 20250820 hx 增加中继转发时延
        while (!pq.empty()) {
            auto [current_dist, u] = pq.top();
            pq.pop();

            if (current_dist > dist[u])
                continue; // 跳过非最优路径
            if (!adj.count(u))
                continue; // 无出边，跳过

            for (const auto& [v, comm_delay] : adj.at(u)) {
                double total_delay = comm_delay;
                if (u != src) {
                    total_delay += transfer_delay; // 增加转发时延
                }

                if (dist[v] > dist[u] + total_delay) {
                    dist[v] = dist[u] + total_delay;
                    prev[v] = u; // 记录v的前驱为u
                    pq.emplace(dist[v], v);
                }
            }
        }

        // 构建结果：计算每个节点的可达性、最短路径
        std::map<VID, DelayResult> result;
        for (const auto& [node, d] : dist) {
            DelayResult dr;
            dr.delay = d;
            dr.reachable = (d < std::numeric_limits<double>::infinity());
            dr.path.clear();

            //// 回溯路径：从node反向找到src，再反转得到正序
            //if (dr.reachable) {
            //    VID curr = node;
            //    while (curr != -1) { // 直到前驱为-1（src的前驱是-1）
            //        dr.path.push_back(curr);
            //        if (curr == src)
            //            break; // 到达源节点则终止
            //        curr = prev[curr];
            //    }
            //    std::reverse(dr.path.begin(), dr.path.end()); // 反转：src->...->node
            //}

            result[node] = dr;
            optresult[src][node] = std::make_tuple(d, dr.reachable, dr.path);
        }
        return result;
    }
};


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
    CommunicationData bestComm;
    double tmpdalay = 99;
    double set_Pt = 0;
    auto& optcomm = c.getSpecificSingleton<OptCommMemory>().value();
    auto&& myvid = c.getSpecificSingleton<VID>().value();

    for (auto&& [id, _commdata, _damage] : c.getNormal<CommunicationData, DamageModel>()) {
        if (_damage.damageLevel == DAMAGE_LEVEL::K || _damage.damageLevel == DAMAGE_LEVEL::KK) {
            continue;
        }
        if ((_commdata.launchdelay + _commdata.receivedelay + _commdata.transdelay ) <
            tmpdalay) {
            bestComm = _commdata;
            tmpdalay = _commdata.launchdelay + _commdata.receivedelay + _commdata.transdelay;
        }
        set_Pt = _commdata.transpower;
    }
    CommuState commstate;
    commstate.time_delay = INFINITY;
    ObstacleInfo obstacletmp;
    auto calculateSNR = [&](const Vector3& pos1, const Vector3& pos2) -> double {
        double SNR = comm->getcommPrPn(pos1, pos2, bestComm);
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
            get<1>(entityInfo).baseInfo.side == c.getSpecificSingleton<SID>().value() &&
            vid != c.getSpecificSingleton<VID>().value())
            uavid.push_back(vid);
    }
    //  1.遍历scannedmemory所有我方车辆，计算是否可以通信，以及误码率和时延，存储到本车的CommuState中,这里的存储信息为DDS直接输入，还未经sensor.tick处理为真实探测
    for (auto&& [vid, entityInfo] : c.getSpecificSingleton<ScannedMemory>().value()) {
        // 计算通信结果并存储

        uavmodel::Vector3 myPos = c.getSpecificSingleton<Coordinate>().value().position;
        uavmodel::Vector3 otherPos = get<1>(entityInfo).position;
        if (get<1>(entityInfo).baseInfo.side == c.getSpecificSingleton<SID>().value()) {
            // commstate.success = true;
            // commstate.time_delay = 10.0;
            // commstate.rate_error = 0.0;
            if (vid == c.getSpecificSingleton<VID>().value()) {
                get<0>(optcomm[myvid][vid]).time_delay = 999;
                get<0>(optcomm[myvid][vid]).success = false;
                get<0>(optcomm[myvid][vid]).rate_error = INFINITY;
            } else {
                double SNR = calculateSNR(myPos, otherPos);
                // 2.计算误码率0~1，判断是否成功通信
                commstate.rate_error = 0.5 * erfc(sqrt(SNR));
                commstate.success = (commstate.rate_error < 1e-5);

                                // 20250816 hx
                get<0>(optcomm[myvid][vid]).success = commstate.success;
                get<0>(optcomm[myvid][vid]).rate_error = commstate.rate_error;
                // 互相通信
                get<0>(optcomm[vid][myvid]).success = commstate.success;
                get<0>(optcomm[vid][myvid]).rate_error = commstate.rate_error;

                if (commstate.success) {
                    // 20250815 hx 修改时延 ：两两时延 传播时延+发射时延+接收时延（各个设备不同car.xml配置）
                    double distance = (myPos - otherPos).norm();
                    commstate.time_delay = distance / c_speed + bestComm.launchdelay + bestComm.receivedelay;
                    get<0>(optcomm[myvid][vid]).time_delay = commstate.time_delay;
                    get<0>(optcomm[vid][myvid]).time_delay = commstate.time_delay;
                }
            }
            commmemory[vid] = commstate; // vid为通信目标车辆的VID，CommuState为与此目标的通信状态
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

    // 20250815 hx 中继通信功能实现 ScannedMemory中不包含自己
    CommunicationAnalyzer analyzer;
    //  根据时延排序
    std::vector<IndexEntry> entries;
    for (auto&& [vid, entityInfo] : c.getSpecificSingleton<ScannedMemory>().value()) {
        // std::cout << vid;
        if (vid == myvid) {
            continue;
        } else {
            auto&& vid_comm = (*(c.getSpecificSingleton<SystemScannedMemoryget>()))[vid];
            for (auto&& [targetid, delaytime] : vid_comm) {
                get<0>(optcomm[vid][targetid]).time_delay = get<0>(delaytime);
                get<0>(optcomm[vid][targetid]).success = (get<0>(delaytime) < 999);
            }
        }
    }
    analyzer.buildGraph(optcomm); // 构建通信拓扑（仅含直接成功的边）

    // 20250820 hx 遍历所有源节点，分析并打印链路
    for (const auto& [src, _] : analyzer.adj) {
        auto delays = analyzer.calculateDelays(src, c, bestComm); // 获取源节点到所有节点的路径信息
    }
}
// 计算绕射常数
} // namespace uavmodel