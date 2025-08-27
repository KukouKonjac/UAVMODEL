#pragma once

#include <Eigen/Dense>
#include <any>
#include <array>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

#include "../framework/componentmanager.hpp"
#include "constant.hpp"
#include "coordinate.hpp"
#include "vector3.hpp"

namespace uavmodel {

namespace command {

enum class COMMAND_TYPE {
    FOLLOWCAR = 0,
    CLIMB = 1,   // 爬升
    DIVE,        // 下降
    LEVELFLIGHT, // 平飞
    HOVER,       // 悬停
    BACK,
    SURROUND, // 盘旋
    RADAR_SWITCH,
    FOLLOW_ROAD,
    SET_ROAD,
    ACTIVATE_INTERFERE,
    // REPAIR,
};

inline size_t NoParamMask =
    size_t(1) << static_cast<int>(COMMAND_TYPE::HOVER) | size_t(1) << static_cast<int>(COMMAND_TYPE::BACK) |
    size_t(1) << static_cast<int>(COMMAND_TYPE::FOLLOWCAR) | size_t(1) << static_cast<int>(COMMAND_TYPE::SET_ROAD);

inline size_t SingleParamMask = size_t(1) << static_cast<int>(COMMAND_TYPE::CLIMB) |
                                size_t(1) << static_cast<int>(COMMAND_TYPE::DIVE) |
                                size_t(1) << static_cast<int>(COMMAND_TYPE::ACTIVATE_INTERFERE);

inline size_t DoubleParamMask = size_t(1) << static_cast<int>(COMMAND_TYPE::LEVELFLIGHT) |
                                size_t(1) << static_cast<int>(COMMAND_TYPE::RADAR_SWITCH) |
                                size_t(1) << static_cast<int>(COMMAND_TYPE::SET_ROAD) |
                                size_t(1) << static_cast<int>(COMMAND_TYPE::SURROUND) /*|
                                 size_t(1) << static_cast<int>(COMMAND_TYPE::REPAIR)*/
    ;

} // namespace command

// vehicle ID
using VID = uint64_t;
// side ID
using SID = uint16_t;

using PLATOONID = uint64_t;

struct Block {
    constexpr static const char* token_list[] = {"length", "width", "height"};
    constexpr operator Vector3() const { return Vector3{length, width, height}; };
    constexpr double operator[](size_t i) const { return (&length)[i]; };
    double& operator[](size_t i) { return (&length)[i]; };
    double length, width, height;
    static Block make() { return Block{}; }
};

struct Sphere {
    double r;
};

/**
 * @brief protection model, used in damage calculation of ammunition
 *
 */
struct ProtectionModel {
    constexpr static const char* token_list[] = {"armor_front",
                                                 "armor_back",
                                                 "armor_side",
                                                 "armor_bottom",
                                                 "armor_top",
                                                 "activeProtectionAmmo",
                                                 "reactiveArmor",
                                                 "coverageRate",
                                                 "jammer",
                                                 "hidden",
                                                 "Interception_probability1",
                                                 "Interception probability2",
                                                 "active_interference_rate",
                                                 "active_interference_distance"};
    // armor thickness of each side
    double armor_front;
    double armor_back;
    double armor_side;
    double armor_bottom;
    double armor_top;
    // counter of active protection ammo
    int activeProtectionAmmo;
    // counter of reactive armor
    int reactiveArmor;
    // rate of surface area which covered by reactive armor
    double coverageRate;
    // the jammer capability
    double jammer;
    // the hidden capability, the founded probability
    double hidden;
    // the Interception probability of Anti tank missiles, rockets
    double Interception_probability1;
    // the Interception probability of AP,HE
    double Interception_probability2;
    // Success rate of active interference
    double active_interference_rate;
    // the distance of active interference
    double active_interference_distance;
    static ProtectionModel make() { return ProtectionModel{}; }
};

// hull
struct Hull {
    Vector3 velocity;
    Vector3 palstance;
    bool out_of_range = false;
};

// cardamage，越小越正常
enum class DAMAGE_LEVEL {
    N = 0,  // 正常
    M = 1,  // 中度毁伤
    K = 2,  // 失去功能
    KK = 3, // 失去结构
};

struct DamageModel {
    constexpr static const char* token_list[] = {"-damageLevel", "maxInfluence", "repairTime_K", "repairTime_KK"};
    DAMAGE_LEVEL damageLevel;
    DAMAGE_LEVEL maxInfluence;
    double repairTime_K;
    double repairTime_KK;
    static DamageModel make() { return DamageModel{DAMAGE_LEVEL::N, DAMAGE_LEVEL::KK, 0, 0}; }
};

// TODO: index to prevent redundant processing?
struct FireEvent {
    // constexpr static const char *token_list[] = {"weaponName", "target", "position", "velocity", "range"};
    std::string weaponName;
    // 目标点的世界坐标
    Vector3 target;
    // 炮弹位置的世界坐标
    Vector3 position;
    // 当前速度矢量的世界坐标
    Vector3 velocity;
    // 发射者和命中点的直线距离
    double range;
    // add by wsb
    // 是否首发
    double isFirst;
    // 参数1
    double param1;
    // 参数2
    double param2;
};

struct Direction {
    constexpr static const char* token_list[] = {"yaw", "pitch"};
    double yaw, pitch;
    double& operator[](size_t index) { return (&yaw)[index]; };
    const double& operator[](size_t index) const { return (&yaw)[index]; };
    static Direction make() { return Direction{}; }
};

struct AngleZone {
    constexpr static const char* token_list[] = {"yawLeft", "yawRight", "pitchUp", "pitchDown"};
    double yawLeft, yawRight, pitchUp, pitchDown;
    double& operator[](size_t index) { return (&yawLeft)[index]; };
    const double& operator[](size_t index) const { return (&yawLeft)[index]; };
    bool containsDirection(const Direction& d) const {
        return d.yaw >= -yawLeft - INF_SMALL && d.yaw <= yawRight + INF_SMALL && d.pitch <= pitchUp + INF_SMALL &&
               d.pitch >= -pitchDown - INF_SMALL;
    }
    static AngleZone make() { return AngleZone{}; }
};

// carfireunit
enum class FIRE_UNIT_STATE {
    FREE,
    LOCK_TARGET, // must have target ID
    LOCK_DIRECTION,
    SINGLE_SHOOT, // must have target ID
    MULTI_SHOOT,  // must have target ID
    SEEK_TARGET,
};

struct Weapon {
    constexpr static const char* token_list[] = {"ammoType", "ammoRemain", "reloadingTime", "-reloadingState",
                                                 "range",    "speed",      "param1",        "param2"};
    std::string ammoType;
    int ammoRemain;
    double reloadingTime;
    double reloadingState; ///< current remain reloading time
    double range;
    double speed;
    // add by wsb:for different weapon type, the param1 and param2 refers to different meaning
    double param1;
    double param2;
    int ammototal;
    static Weapon make() {
        Weapon tmp;
        tmp.reloadingState = 0;
        return tmp;
    }
};

struct FireUnit {
    constexpr static const char* token_list[] = {"-state",      "-data", "fireZone", "rotateZone", "-presentDirection",
                                                 "rotateSpeed", "weapon"};
    FIRE_UNIT_STATE state;
    double data;                ///< target ID or angle
    AngleZone fireZone;         ///< [yawLeft, yawRight, pitchUp, pitchDown]
    AngleZone rotateZone;       ///< [yawLeft, yawRight, pitchUp, pitchDown]
    Direction presentDirection; ///< [yaw, pitch]
    Direction rotateSpeed;      ///< [yaw, pitch]
    Weapon weapon;
    static FireUnit make() {
        FireUnit tmp;
        tmp.state = FIRE_UNIT_STATE::FREE;
        tmp.data = 0;
        tmp.presentDirection = Direction{0, 0};
        return tmp;
    }
};

// carsensor
struct SensorData {
    constexpr static const char* token_list[] = {"type",
                                                 "detectrange",
                                                 "detectprobability",
                                                 "target_positioning_accuracy",
                                                 "min_detect_height",
                                                 "max_detect_height",
                                                 "laser_mindetectrange",
                                                 "laser_maxdetectrange"};
    std::string type;
    double detectrange;
    double detectprobability;
    double target_positioning_accuracy;
    double min_detect_height;
    double max_detect_height;
    double laser_mindetectrange;
    double laser_maxdetectrange;
    static SensorData make() { return SensorData{}; }
};

// carcommunication
struct CommunicationData {
    constexpr static const char* token_list[] = {"type",       "launchdelay", "receivedelay",
                                                 "transdelay", "jumpcount",   "transpower"};
    std::string type;
    double launchdelay;
    double receivedelay;
    double transdelay;
    double transpower;
    static CommunicationData make() {
        CommunicationData tmp;
        tmp.launchdelay = 0.1;
        tmp.receivedelay = 0.1;
        tmp.transdelay = 0.1;
        tmp.transpower = 1.0;
        return CommunicationData{};
    }
};

struct BaseInfo {
    constexpr static const char* token_list[] = {"type",
                                                 "id",
                                                 "side",
                                                 "damageLevel",
                                                 "jammer",
                                                 "hidden",
                                                 "active_interference_rate",
                                                 "active_interference_distance",
                                                 "platoonid"};
    enum class ENTITY_TYPE {
        CAR,
        TANK,
        SUPPORTCAR,
        // UAV,
        UGV,
        UAV,
        UNKNOWN = -1,
    } type;
    VID id;
    SID side;
    DAMAGE_LEVEL damageLevel;
    double jammer;
    double hidden;
    double active_interference_rate;
    double active_interference_distance;
    VID platoonid;
    static BaseInfo make() { return BaseInfo{}; }
};

struct EntityInfo {
    // constexpr static const char *token_list[] = {"position", "velocity", "baseInfo"};
    Vector3 position;
    Vector3 velocity;
    BaseInfo baseInfo;
    // double lastScanned;
};

// component ID
using CID = size_t;

struct ScannedMemory : public std::map<VID, std::tuple<double, EntityInfo>> {};
// add by wsb:add system scanned memory, double存入通信时延
struct SystemScannedMemory : public std::map<VID, std::tuple<double, EntityInfo>> {};

struct CommuState {
    bool success;      // 是否成功
    double time_delay; // 时延
    double rate_error; // 误码率
};
struct CommunicaionMemory : public std::map<VID, CommuState> {};

struct SystemScannedMemoryget : public std::map<VID, std::map<VID, std::tuple<double, EntityInfo>>> {};
// struct SystemScannedMemoryget : public std::map<VID, std::map<VID, std::tuple<double, EntityInfo>>> {};

// 20250820 hx 优化时延结构OptCommMemory和OptCommResult
struct OptCommMemory : public std::map<VID, std::map<VID, std::tuple<CommuState>>> {};
struct OptCommResult : public std::map<VID, std::map<VID, std::tuple<double, bool, std::vector<VID>>>> {};

struct WheelMotionParamList {
    constexpr static const char* token_list[] = {"-angle", "LENGTH", "MAX_ANGLE", "ROTATE_SPEED", "MAX_LINEAR_SPEED",
                                                 "MAX_FRONT_ACCELERATION", "MAX_BRAKE_ACCELERATION",
                                                 "MAX_LATERAL_ACCELERATION",
                                                 // below add by wsb
                                                 "OIL_REMAIN", "OIL_CONSUMPTION", "MAX_CLIMBING_ACCELERATION"};
    // 车轮转角，右为正
    double angle;
    // 前后轴距
    double LENGTH;
    // 前轮最大转角约束
    double MAX_ANGLE;
    // 车轮转动速度约束
    double ROTATE_SPEED;
    // 最大直线速度约束
    double MAX_LINEAR_SPEED;
    // 最大前向加速度约束
    double MAX_FRONT_ACCELERATION;
    // 最大减速加速度约束
    double MAX_BRAKE_ACCELERATION;
    // 最大转弯向心加速度(侧向加速度)约束
    double MAX_LATERAL_ACCELERATION;
    // 剩余油量
    double OIL_REMAIN;
    // 油耗,百公里耗油量
    double OIL_CONSUMPTION;
    // 最大爬坡加速度约束
    double MAX_CLIMBING_ACCELERATION;
    static WheelMotionParamList make() {
        WheelMotionParamList tmp;
        tmp.angle = 0;
        return tmp;
    }
};
struct QuadrotorMotionParamList {
    constexpr static const char* token_list[] = {"MAX_CLIMB_SPEED",
                                                 "MAX_DIVE_SPEED",
                                                 "MAX_LEVELFLY_SPEED",
                                                 "CHARGING_EFFICIENTY"
                                                 "BATTERY",
                                                 "MAX_FLY_TIME "
                                                 "ROTATE_SPEED",
                                                 "LENGTH_D",
                                                 "F_MAX",
                                                 "CT",
                                                 "CM",
                                                 "J0",
                                                 "JXX",
                                                 "JYY",
                                                 "JZZ",
                                                 "M"
                                                 "MAX_CLIMB_HEIGHT"
                                                 "TIED_WITH_CAR"
                                                 "MAX_CONTROL_RANGE"};
    // 最大爬升速度
    double MAX_CLIMB_SPEED;
    // 最大下降速度（垂直）
    double MAX_DIVE_SPEED;
    // 最大平飞速度
    double MAX_LEVELFLY_SPEED;
    // 当前电量支持飞行时间（单位：秒）
    double BATTERY;
    // 最大飞行时间（单位：秒）
    double MAX_FLY_TIME;
    // 充电效率（最大飞行时间/充电时间）
    double CHARGING_EFFICIENTY;
    // 最大旋转角速度
    double ROTATE_SPEED;
    // 升力系数
    double CT;
    // 力矩系数
    double CM;
    // 机臂长度
    double LENGTH_D;
    // 最大力
    double F_MAX;
    // 待定
    // 电机和螺旋桨的转动惯量
    double J0;
    // X方向转动惯量
    double JXX;
    // Y方向转动惯量
    double JYY;
    // Z方向转动惯量
    double JZZ;
    // 质量
    double M;
    // 最大爬升高度
    double MAX_CLIMB_HEIGHT;
    // 是否绑定车辆
    VID TIED_WITH_CAR;
    double MAX_CONTROL_RANGE;
    Eigen::Vector4d force;
    Eigen::Vector4d w_rotor;
    static QuadrotorMotionParamList make() {
        QuadrotorMotionParamList tmp;
        tmp.BATTERY = 2400;
        tmp.J0 = 1.01e-5;
        tmp.JXX = 4.212e-3;
        tmp.JYY = 4.212e-3;
        tmp.JZZ = 8.255e-3;
        tmp.M = 0.8;
        tmp.CT = 2.168e-6;
        tmp.CM = 2.136e-8;
        tmp.TIED_WITH_CAR = 0;
        return tmp;
    }
};
struct HitEventQueue : public std::vector<FireEvent> {};

struct FireEventQueue : public std::vector<FireEvent> {};

struct CommandBuffer : public std::map<command::COMMAND_TYPE, std::any> {};

struct EventBuffer : public std::unordered_map<std::string, std::any> {};

struct PathPlanningModel {
    std::vector<Vector3> route;
    Vector3 prePoint;
    size_t nextPoint;
};
struct SurroundState {
    bool inOrbit = false;
    bool isSurrounding = false;                // 已启动环绕
    bool hasCompletedFirstRevolution = false;  // 第一圈完成（启动扩展）
    bool hasCompletedSecondRevolution = false; // 外圈稳定飞行一圈完成

    uavmodel::Vector3 centerPosition; // 圆心位置（固定）
    double targetRadius = 0.0;        // 初始半径
    double finalRadius = 0.0;         // 扩展目标半径 = 2 * targetRadius

    double angleTraversed = 0.0;                 // 第一圈累计角度
    double secondRevolutionAngleTraversed = 0.0; // 第二圈累计角度

    double lastAngle = 0.0; // 上次角度，用于差值计算
    int completedCircles = 1;     // ✅ 新增：已完成的外圈数
    double lastRadialError = 0.0; // ✅ 新增：用于 PD 控制的微分项
};

using Components = ComponentManager<
    SingletonComponent<Coordinate, DamageModel, CommandBuffer, EventBuffer, HitEventQueue, FireEventQueue,
                       WheelMotionParamList, QuadrotorMotionParamList, ScannedMemory, Sphere, Hull, SID, VID, PLATOONID,
                       PathPlanningModel, SystemScannedMemory, SystemScannedMemoryget, CommunicaionMemory,
                       SurroundState, OptCommMemory, OptCommResult>,
    NormalComponent<Coordinate, DamageModel, Block, ProtectionModel, FireUnit, SensorData, CommunicationData>>;

}; // namespace uavmodel