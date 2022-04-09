//
// Created by djcupo on 7/31/17.
//

#include "kilotrack_updater.h"
#include "argos3/core/utility/logging/argos_log.h"


void KiloTrackUpdater::Init(TConfigurationNode& t_tree){
    std::string strHost = "localhost";
    SInt32 nPort = 22222;

    GetNodeAttributeOrDefault(t_tree, "host", strHost, strHost);
    GetNodeAttributeOrDefault(t_tree, "port", nPort, nPort);

    m_tThreadInfo.cConnection.Connect(strHost, nPort);
    m_tThreadInfo.run = true;
    m_cSendThread = std::thread(ThreadCallback, &m_tThreadInfo);
}

TRobotInfoList KiloTrackUpdater::Update(){
    std::map<UInt32, TPose> cPoses;
    TRobotInfoList tRobotInfoList;
    std::string strType="Kilobot", strName;
    CVector3 cPosition;
    CQuaternion cRotation;
    cPosition.SetZ(0);

    std::unique_lock<std::mutex> lock(m_tThreadInfo.cPosesMutex);
    cPoses = m_tThreadInfo.cPoses;
    m_tThreadInfo.cPoses.clear();
    lock.unlock();

    for(auto pair : cPoses){
        strName = strType + "_" + std::to_string(pair.first);
        cPosition.SetX(pair.second.Position.GetX());
        cPosition.SetY(pair.second.Position.GetY());
        cRotation.FromAngleAxis(pair.second.Rotation, CVector3::Z);
        tRobotInfoList.emplace_back(strType, strName, cPosition, cRotation);
    }

    return tRobotInfoList;
}

void KiloTrackUpdater::Destroy(){
    m_tThreadInfo.run = false;
    m_cSendThread.join();

}

void KiloTrackUpdater::ThreadCallback(TThreadInfo* t_thread_info){
    LOG.AddThreadSafeBuffer();
    LOGERR.AddThreadSafeBuffer();
    std::unique_lock<std::mutex> lock(t_thread_info->cPosesMutex, std::defer_lock);
    CByteArray cReadMessage;

    while(t_thread_info->run){
        if(t_thread_info->cConnection.IsConnected() &&
           t_thread_info->cConnection.MessageAvalible()){
            cReadMessage.Clear();
            if(t_thread_info->cConnection.ReceiveByteArray(cReadMessage)){
                CByteArray mes = cReadMessage;
                while(cReadMessage.Size() >= sizeof(UInt32)*2 + sizeof(Real)*3+ 12) {
                    UInt32 id = cReadMessage.PopFront<UInt32>();
                    TPose tPose = TPose();
                    tPose.Position.SetX(cReadMessage.PopFront<Real>());
                    tPose.Position.SetY(cReadMessage.PopFront<Real>());
                    tPose.Rotation.SetValue(cReadMessage.PopFront<Real>());
                    cReadMessage.PopFront<UInt32>();

                    lock.lock();
                    t_thread_info->cPoses[id] = tPose;
                    lock.unlock();
                }
            }
        }
    }
}

REGISTER_TRACKING_UPDATER(KiloTrackUpdater, "kilotrack_updater");