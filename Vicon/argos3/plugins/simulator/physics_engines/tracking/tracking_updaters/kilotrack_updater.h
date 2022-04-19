//
// Created by djcupo on 7/31/17.
//

#ifndef VICON_KILOTRACK_UPDATER_H
#define VICON_KILOTRACK_UPDATER_H

#include <argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/tracking_updater.h>
#include <argos3/plugins/utility/networking/moveable_tcp_socket.h>
#include <mutex>
#include <thread>

using namespace argos;

class KiloTrackUpdater : public CTrackingUpdater{
public:
    typedef struct SPose{
        CVector2 Position;
        CRadians Rotation;
    } TPose;
    typedef struct SThreadInfo{
        bool run;
        std::map<UInt32, TPose> cPoses;
        std::mutex cPosesMutex;
        CMTCPSocket cConnection;
    } TThreadInfo;

public:
    virtual void Init(TConfigurationNode& t_tree);

    virtual TRobotInfoList Update();

    virtual void Reset(){}

    virtual void Destroy();

    void static ThreadCallback(TThreadInfo* t_thread_info);

private:
    TThreadInfo m_tThreadInfo;
    std::thread m_cSendThread;

};

#endif //VICON_KILOTRACK_UPDATER_H
