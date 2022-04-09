#ifndef TRACKING_ENGINE_H
#define TRACKING_ENGINE_H

#include <map>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/utility/controller_assignment/controller_assigner.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_model.h>
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/tracking_updater.h>


class CTrackingModel;

using namespace argos;

class CTrackingEngine : public CPhysicsEngine {
 
public:
    typedef std::map<std::string, CTrackingModel*> EntityMap;
    typedef std::vector<CTrackingUpdater*> TUpdaters;
    typedef std::map<std::string, std::pair<Real, UInt16>> RABOptions;
    typedef std::vector<std::string> TTypes;

    static TTypes ValidTypes;

    static bool IsValidType(const std::string& str_type);

    static std::string GetRobotTypeFromName(const std::string& str_name);

public:

    void Init(TConfigurationNode& t_tree) override;

    void InitUpdaters(TConfigurationNode& t_tree);

    void Reset() override;

    void Destroy() override;

    void Update() override;

    void PostSpaceInit() override;

    size_t GetNumPhysicsModels() override;

    bool AddEntity(CEntity& c_entity) override;

    bool RemoveEntity(CEntity& c_entity) override;

    void CheckIntersectionWithRay(TEmbodiedEntityIntersectionData& t_data,
       const CRay3& c_ray) const override;

    void CreateRobot(const std::string& str_type, 
                     const std::string& str_name,
                     const std::string& str_options = "");

    bool HasRobot(const std::string& str_name);

    inline const std::string GetController(const std::string& str_type){
        return m_cControllerAssigner.GetController(str_type);
    }

private:
    TUpdaters m_tUpdaters;
    EntityMap m_tEntityMap;

    CControlerAssigner m_cControllerAssigner;

    /**
     * Updates a robot based on the information from the Vicon subsystem.
     */
    void UpdateRobot(const TRobotInfo& t_robot_info);

    RABOptions m_tRABOptions;
};

#endif

