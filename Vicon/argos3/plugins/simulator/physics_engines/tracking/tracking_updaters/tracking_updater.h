#ifndef TRACKING_UPDATER_H
#define TRACKING_UPDATER_H

#include <vector>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/configuration/base_configurable_resource.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/plugins/factory.h>

using namespace argos;

typedef struct SRobotInfo{
    const std::string strType;
    const std::string strName;
    const CVector3 cTrans;
    const CQuaternion cQuat;
    const std::string strOptions;

    SRobotInfo(std::string str_type,
               std::string str_name,
               const CVector3& c_origin,
               const CQuaternion& c_rotation,
               std::string str_options = ""):
        strType(std::move(str_type)),
        strName(std::move(str_name)),
        cTrans(c_origin),
        cQuat(c_rotation),
        strOptions(std::move(str_options)){};

} TRobotInfo;

typedef std::vector<TRobotInfo> TRobotInfoList;


class CTrackingUpdater : public  CBaseConfigurableResource{
public:

    virtual void Init(TConfigurationNode& t_tree) override = 0;
    virtual void Reset() override = 0;
    virtual void Destroy() override = 0;


    /**
     * Called by the tracking engine on every update. 
     * @return List of entities to update and the new location, rotation, and options.
     */
    virtual TRobotInfoList Update() = 0;

};

typedef CFactory<CTrackingUpdater> TFactoryTrackingUpdater;

#define REGISTER_TRACKING_UPDATER(CLASSNAME, LABEL) REGISTER_SYMBOL(CTrackingUpdater, CLASSNAME, LABEL,"","","","","")


#endif