#include <argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/spam_updater.h>
#include <argos3/core/utility/logging/argos_log.h>

void CSpamUpdater::Init(TConfigurationNode& t_tree){
    GetNodeAttribute(t_tree, "robot", m_strRobotType);
    m_snLimit = -1;
    GetNodeAttributeOrDefault(t_tree, "limit", m_snLimit, m_snLimit);
    m_unRequest = 0;
}

/****************************************/
/****************************************/

TRobotInfoList CSpamUpdater::Update(){
    m_unRequest += 1;
    if(m_snLimit == -1 || m_unRequest <= m_snLimit){
        TRobotInfoList tRobotInfoList;
        CVector3 cOrigin(0,0,0);
        CQuaternion cRotation(1,0,0,0);
        std::string strName;
        strName =  m_strRobotType + "_" + std::to_string(m_unRequest);

        TRobotInfo tRobotInfo(m_strRobotType, strName , cOrigin, cRotation);
        tRobotInfoList.push_back(tRobotInfo);
        return tRobotInfoList;
    }
    return TRobotInfoList();
}

/****************************************/
/****************************************/

void CSpamUpdater::Destroy(){

} 

REGISTER_TRACKING_UPDATER(CSpamUpdater, "spam_updater");
