#include <algorithm>
#include <array>

#include "uavbuilder.h"

#include "framework/component.hpp"
#include "src/extern/rapidxml-1.13/rapidxml.hpp"
#include "tools/metatools.hpp"
#include "tools/myassert.hpp"
#include "tools/seterror.hpp"

#include "damage/uavdamage.h"
#include "firecontrolsystem/uavfcs.hpp"
#include "hull/uavhull.h"
#include "protection/uavprotection.h"
#include "sensors/uavsensor.h"
#include "toolsystem/ballisticsystem.hpp"
#include "toolsystem/hitsystem.hpp"

namespace {

using namespace rapidxml;
using namespace std;
using namespace uavmodel;
using namespace uavmodel::component;
using namespace uavmodel::mymeta;

template <typename... Ty>
struct NameTable {
    std::array<std::string_view, sizeof...(Ty)> table;
    template <typename T>
    constexpr std::string_view getName() const {
        return table[get_type_list_index<T, Ty...>::value];
    }
};

constexpr NameTable<WheelMotionParamList, QuadrotorMotionParamList, Coordinate, DamageModel, Block, ProtectionModel, FireUnit, SensorData,
                    CommunicationData>
    nameTable{{
        "WheelMotionParamList",
        "QuadrotorMotionParamList",
        "Coordinate",
        "DamageModel",
        "Block",
        "ProtectionModel",
        "FireUnit",
        "SensorData",
        "CommunicationData",
    }};

class CStyleString {
  public:
    char* s;
    CStyleString(size_t n) : s(new char[n]){};
    CStyleString(const CStyleString& s) = delete;
    void operator=(const CStyleString& s) = delete;
    CStyleString(const std::string& str) : s(new char[str.size() + 5]) { memcpy(s, str.c_str(), str.size() + 1); };
    ~CStyleString() { delete[] s; };
};

template <typename Ty>
struct loadComponent;

template <typename... Ty>
struct loadComponent<NormalComponent<Ty...>> {
    static void load(size_t ID, rapidxml::xml_node<char>* component, uavmodel::Components::Modifier& handle) {
        bool match = (... || [&]() {
            // TODO: use set instead?
            if (component->name() == nameTable.getName<Ty>()) {
                handle.addNormalComponents<Ty>(ID, componentDeserialize<Ty>(component));
                return true;
            }
            return false;
        }());
        my_assert(match, "Invalid Component Named "s + component->name());
    }
};

template <typename... Ty>
struct loadComponent<SingletonComponent<Ty...>> {
    static void load(rapidxml::xml_node<char>* root, uavmodel::Components::Modifier& handle) {
        int tmp[] = {[&]() {
            if (auto p = root->first_node(nameTable.getName<Ty>().data(), nameTable.getName<Ty>().size()); p != 0) {
                handle.addSingletonComponents<Ty>(componentDeserialize<Ty>(p));//反序列化
            }
            return 0;
        }()...};
    }
};

// void loadComponent(size_t ID, rapidxml::xml_node<char>* component, carPhyModel::Components::Modifier& handle){
//     // name must be a string like "Name"sv to enable operator== between string
//     if(component->name() == "Coordinate"sv){
//         handle.addNormalComponents<Coordinate>(ID, componentDeserialize<Coordinate>(component));
//     }else if(component->name() == "ProtectionModel"sv){
//         handle.addNormalComponents<ProtectionModel>(ID, componentDeserialize<ProtectionModel>(component));
//     }else if(component->name() == "DamageModel"sv){
//         handle.addNormalComponents<DamageModel>(ID, componentDeserialize<DamageModel>(component));
//     }else if(component->name() == "FireUnit"sv){
//         handle.addNormalComponents<FireUnit>(ID, componentDeserialize<FireUnit>(component));
//     }else if(component->name() == "SensorData"sv){
//         handle.addNormalComponents<SensorData>(ID, componentDeserialize<SensorData>(component));
//     }else throw BadComponentName("do not have a component named "s + component->name());
// }

template <typename Tar, typename... Dependencies>
struct Restriction {
    static std::string check(Components& c) {
        for (auto&& [id, e] : c.getNormal<Tar>()) {
            bool fit = (... && c.getSpecificNormal<Dependencies>(id).has_value());
            if (!fit) {
                std::string errorInfo = std::string(nameTable.getName<Tar>()) + " need dependencies ";
                int tmp[] = {[&]() {
                    if (!c.getSpecificNormal<Dependencies>(id).has_value()) {
                        errorInfo += std::string(nameTable.getName<Dependencies>()) + ", ";
                    }
                    return 0;
                }()...};
                errorInfo += "but not found\n";
                return errorInfo;
            }
        }
        return "";
    }
};

// template <typename Tar, typename... Dependencies>
// struct WeakRestriction {
//     static std::string check(Components& c) {
//         for (auto&& [id, e] : c.getNormal<Tar>()) {
//             bool fit = (... && c.getSpecificNormal<Dependencies>(id).has_value());
//             if (!fit) {
//                 int tmp[] = {[&](){
//                     if (!c.getSpecificNormal<Dependencies>(id).has_value()){
//                     }
//                     return 0;
//                 }()...};
//             }
//         }
//         return "";
//     }
// };

template <typename... Restrictions>
struct RestrictionList {
    static std::string check(Components& c) { return (... + Restrictions::check(c)); }
};

void checkRestriction(Components& c) {
    using namespace uavmodel;
    static RestrictionList<
        Restriction<Block, Coordinate>,
        Restriction<ProtectionModel, Block>, 
        Restriction<DamageModel, Block>,
        Restriction<SensorData, DamageModel>, 
        Restriction<FireUnit, DamageModel>
    > checker;
    auto s = checker.check(c);
    if (!s.empty()) {
        error(s);
    }
}

} // namespace

namespace uavmodel {

void UavBuilder::buildFromSource(const std::string& srcXML, UavModel& model, bool check) {
    static struct InitJob{
        InitJob() { 
            auto& system = UavModel::systems;
            system.push_back(std::make_unique<PrepareSystem>());

            system.push_back(std::make_unique<SensorSystem>());
            system.push_back(std::make_unique<FireControlSystem>());

            system.push_back(std::make_unique<BallisticSystem>());
            system.push_back(std::make_unique<HitSystem>());

            system.push_back(std::make_unique<DamageSystem>());

            system.push_back(std::make_unique<HullSystem>());

            system.push_back(std::make_unique<OutputSystem>());
        }
    } initJob;

    // 初始化xml解析器
    xml_document<> doc;
    CStyleString s(srcXML);
    doc.parse<parse_default>(s.s);
    auto root = doc.first_node("car");

    if (auto handle = model.components.getModifier()) {
        handle.addSingletonComponents<CommandBuffer, EventBuffer, DamageModel, Coordinate, HitEventQueue,
                                      FireEventQueue, ScannedMemory, Hull, PathPlanningModel, SystemScannedMemory, SystemScannedMemoryget>({}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {});
        handle.addSingletonComponents<DamageModel>({DAMAGE_LEVEL::N, DAMAGE_LEVEL::KK, 1e10, 1e10});
        handle.addSingletonComponents<SID, VID>(0, 0);

        loadComponent<SingletonComponent<WheelMotionParamList>>::load(root, handle);
        loadComponent<SingletonComponent<QuadrotorMotionParamList>>::load(root, handle);
        //add by wsb
        loadComponent<SingletonComponent<DamageModel>>::load(root, handle);

        for (auto entity = root->first_node("entity"); entity; entity = entity->next_sibling("entity")) {
            auto ID = handle.newEntity();
            for (auto component = entity->first_node(0); component; component = component->next_sibling()) {
                loadComponent<NormalComponent<Block, ProtectionModel, DamageModel, FireUnit, SensorData,
                                              CommunicationData,
                                              Coordinate>>::load(ID, component, handle);
            }
        }
    }

    if (check) {
        checkRestriction(model.components);
    }
}

} // namespace carphymodel