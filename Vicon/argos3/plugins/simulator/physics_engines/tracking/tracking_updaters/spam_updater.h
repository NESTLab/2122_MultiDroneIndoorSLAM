#ifndef SPAM_UPDATER_H
#define SPAM_UPDATER_H

#include <argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/tracking_updater.h>

using namespace argos;

class CSpamUpdater : public CTrackingUpdater{
public: 
    void Init(TConfigurationNode& t_tree) override;

    TRobotInfoList Update() override;

    void Reset() override {}

    void Destroy() override;

private:
    std::string m_strRobotType;
    UInt16 m_unRequest;
    SInt16 m_snLimit;

};

#endif