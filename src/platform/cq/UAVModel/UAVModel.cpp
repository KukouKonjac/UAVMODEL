#include <format>
#include <string>
#include <chrono>

#include "UAVModel.h"
#include "src/model/uavbuilder.h"
#include "src/model/tools/constant.hpp"
#include "src/model/tools/rand.hpp"
#include "src/model/environment/wsf.hpp"

namespace {

using namespace std;
using namespace uavmodel::command;

constexpr inline double rate = 111000.;//坐标转换

uavmodel::Vector3 locationTrans(const UAVModel::Location& base, const UAVModel::Location& location) {
    return {
        rate * (location.latitude - base.latitude),
        rate * (location.longitude - base.longitude) * cos(uavmodel::DEG2RAD(base.latitude)),
        base.altitude - location.altitude,
    };
}
UAVModel::Location positionTrans(const UAVModel::Location& base, const uavmodel::Vector3& position) {
    return {
        .longitude = position.y / (rate * cos(uavmodel::DEG2RAD(base.latitude))) + base.longitude,
        .latitude = position.x / rate + base.latitude,
        .altitude = base.altitude - position.z,
    };
}

string getLibDir() {
    string library_dir_;
#ifdef _WIN32
    HMODULE module_instance = _AtlBaseModule.GetModuleInstance();
    char dll_path[MAX_PATH] = {0};
    GetModuleFileNameA(module_instance, dll_path, _countof(dll_path));
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    _splitpath_s(dll_path, drive, dir, fname, ext);
    library_dir_ = drive + std::string(dir) + "\\";
#else
    Dl_info dl_info;
    CSModelObject* (*p)() = &CreateModelObject;
    if (0 != dladdr((void*)(p), &dl_info)) {
        library_dir_ = std::string(dl_info.dli_fname);
        library_dir_ = library_dir_.substr(0, library_dir_.find_last_of('/'));
        library_dir_ += "/";
    }
#endif
    return library_dir_;
}

} // namespace

bool UAVModel::Init(const std::unordered_map<std::string, std::any>& value) {
    if (auto it = value.find("filePath"); it != value.end()) {
        uavmodel::UavBuilder::buildFromFile(any_cast<std::string>(it->second), model);
    } else {
        uavmodel::UavBuilder::buildFromFile(getLibDir() + "uav.xml", model);
    }
    Location tmp{0, 0, 0};
    tmp.longitude = std::any_cast<double>(value.find("longitude")->second);
    tmp.latitude = std::any_cast<double>(value.find("latitude")->second);
    tmp.altitude = std::any_cast<double>(value.find("altitude")->second);
    {
        std::lock_guard<std::mutex> lock(initLock);
        myVID = VIDCounter++;
        if (!myVID) {
            // location of car 0 is base location.
            // CQ will not release dll when restart, but has no unexpected affect
            location = tmp;
            if (auto it = value.find("demFilePath"); it != value.end()) {
                auto env = std::make_unique<wsfplugin::WSFEnvironment>();
                auto filePath = any_cast<std::string>(it->second);
                env->init(filePath, tmp.longitude, tmp.latitude);
                uavmodel::EnvironmentInfoAgent::changeEnvironmentSupplier(std::move(env));
            }
        }
    }
    auto& buffer = model.components.getSpecificSingleton<uavmodel::EventBuffer>().value();
    buffer.emplace("longitude", tmp.longitude);
    buffer.emplace("latitude", tmp.latitude);
    model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position = locationTrans(location, tmp);
    state_ = CSInstanceState::IS_RUNNING;
    return true;
}

bool UAVModel::Tick(double time) {
    // time: ms -> s
    model.tick(time / 1000);

    auto& buffer = model.components.getSpecificSingleton<uavmodel::EventBuffer>();
    buffer->emplace("VID", getVID());
    uavmodel::Vector3 tmp = model.components.getSpecificSingleton<uavmodel::Coordinate>().value().position;
    EntityInfo info;
    info.baseInfo = BaseInfo{
        getVID(),
        GetForceSideID(),
        static_cast<uint16_t>(uavmodel::BaseInfo::ENTITY_TYPE::UAV),
        static_cast<uint16_t>(model.components.getSpecificSingleton<uavmodel::DamageModel>()->damageLevel),
        get<1>(*model.components.getNormal<uavmodel::ProtectionModel>().begin()).jammer,//约定第一个entity反映整体特征
        get<1>(*model.components.getNormal<uavmodel::ProtectionModel>().begin()).hidden,//约定第一个entity反映整体特征
        get<1>(*model.components.getNormal<uavmodel::ProtectionModel>().begin()).active_interference_rate,//约定第一个entity反映整体特征
        get<1>(*model.components.getNormal<uavmodel::ProtectionModel>().begin()).active_interference_distance,//约定第一个entity反映整体特征
    };
    if (info.baseInfo.damageLevel >= static_cast<uint16_t>(uavmodel::DAMAGE_LEVEL::K)) {
        state_ = CSInstanceState::IS_DESTROYED;
    }
    info.position = model.components.getSpecificSingleton<uavmodel::Coordinate>()->position;
    info.velocity = model.components.getSpecificSingleton<uavmodel::Hull>()->velocity;
    
    buffer->emplace("EntityInfoOut", info.ToValueMap());
    Location tmpl = positionTrans(location, info.position);
    double oilremain = model.components.getSpecificSingleton<uavmodel::WheelMotionParamList>()->OIL_REMAIN;
    // deg
    buffer->emplace("longitude", tmpl.longitude);
    buffer->emplace("altitude", tmpl.altitude);
    buffer->emplace("latitude", tmpl.latitude);
    auto attitude = uavmodel::Quaternion::fromCompressedQuaternion(
                        model.components.getSpecificSingleton<uavmodel::Coordinate>().value().attitude)
                        .getEuler();
    // deg
    buffer->emplace("roll", uavmodel::RAD2DEG(attitude.x));
    buffer->emplace("pitch", uavmodel::RAD2DEG(attitude.y));
    buffer->emplace("yaw", uavmodel::RAD2DEG(attitude.z));
    auto velocity = model.components.getSpecificSingleton<uavmodel::Hull>().value().velocity;
    buffer->emplace("velocity_x", velocity.x);
    buffer->emplace("velocity_y", velocity.y);
    buffer->emplace("oil_remain", oilremain);

    std::vector<std::any> scannedInfoOut;
    for (auto& info : model.components.getSpecificSingleton<uavmodel::ScannedMemory>().value()) {
        //判断info中的tuple中的double参数是否==0
        if (std::get<0>(info.second) == 0.)
        {
            scannedInfoOut.emplace_back(EntityInfo(std::get<1>(info.second)).ToValueMap());
        }
    }
    buffer->emplace("scannedInfoOut", std::move(scannedInfoOut));
    return true;
}

bool UAVModel::SetInput(const std::unordered_map<std::string, std::any>& value) {
    if (auto it = value.find("EntityInfo"); it != value.end()) {
        auto& v = it->second;
        EntityInfo tmp;
        tmp.FromValueMap(any_cast<CSValueMap>(v));
        uavmodel::VID ID = tmp.baseInfo.id;
        get<1>((*(model.components.getSpecificSingleton<uavmodel::ScannedMemory>()))[ID]) = tmp;
    }
    if (auto it = value.find("FireData"); it != value.end()) {
        uavmodel::VID ID = any_cast<uavmodel::VID>(value.find("FireID")->second);
        auto& v = it->second;
        if (ID != getVID()) {
            FireEvent tmp;
            tmp.FromValueMap(any_cast<CSValueMap>(v));
            model.components.getSpecificSingleton<uavmodel::FireEventQueue>()->push_back(tmp);
        }
    }
    if (auto it = value.find("Command"); it != value.end()) {
        auto& v = it->second;
        auto command = static_cast<COMMAND_TYPE>(std::any_cast<uint64_t>(v));
        auto& buffer = model.components.getSpecificSingleton<uavmodel::CommandBuffer>();
        double param1 = 0., param2 = 0.;
        if (auto it = value.find("Param1"); it != value.end()) {
            param1 = std::any_cast<double>(it->second);
        }
        if (auto it = value.find("Param2"); it != value.end()) {
            param2 = std::any_cast<double>(it->second);
        }
        WriteLog(std::format("UAVModel model receive command {}({},{})", (uint64_t)command, param1, param2), 1);
        buffer->emplace(command, make_tuple(param1, param2));
    }
    return true;
}

std::unordered_map<std::string, std::any>* UAVModel::GetOutput() {
    std::get<0>(model.components.getSingleton<uavmodel::VID>()) = getVID();
    std::get<0>(model.components.getSingleton<uavmodel::SID>()) = GetForceSideID();
    auto& buffer = model.components.getSpecificSingleton<uavmodel::EventBuffer>().value();
    buffer.emplace("ForceSideID", GetForceSideID());
    buffer.emplace("ModelID", GetModelID());
    buffer.emplace("InstanceName", GetInstanceName());
    buffer.emplace("ID", GetID());
    buffer.emplace("State", uint16_t(GetState()));
    return &buffer;
}

UAVMODEL_EXPORT CSModelObject* CreateModelObject() {
    CSModelObject* model = new UAVModel();
    return model;
}

UAVMODEL_EXPORT void DestroyMemory(void* mem, bool is_array) {
    if (is_array) {
        delete[] ((UAVModel*)mem);
    } else {
        delete ((UAVModel*)mem);
    }
}
