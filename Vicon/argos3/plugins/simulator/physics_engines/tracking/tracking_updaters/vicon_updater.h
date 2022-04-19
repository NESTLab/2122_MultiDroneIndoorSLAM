#ifndef VICON_UPDATER_H
#define VICON_UPDATER_H

#include "DataStreamClient.h"
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/tracking_updater.h>

using namespace argos;

namespace ViconSDK = ViconDataStreamSDK::CPP;

class CViconUpdater : public CTrackingUpdater{
public: 
    void Init(TConfigurationNode& t_tree) override;

    TRobotInfoList Update() override;

    void Reset() override {}

    void Destroy() override;

    static void GetRobotTypeFromName(const std::string& str_sub_name,
                                     std::string& str_type,
                                     std::string& str_name,
                                     std::string& str_options);
private:
    ViconSDK::Client m_cClient;

};

#endif