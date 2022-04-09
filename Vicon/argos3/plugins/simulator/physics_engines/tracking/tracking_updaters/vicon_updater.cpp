#include <argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/vicon_updater.h>
#include <argos3/core/utility/logging/argos_log.h>


void CViconUpdater::Init(TConfigurationNode& t_tree){
    std::string strHost = "192.168.1.211";
    SInt32 nPort = 801;
    GetNodeAttributeOrDefault(t_tree, "host", strHost, strHost);
    GetNodeAttributeOrDefault(t_tree, "port", nPort, nPort);

    struct timespec sSleep;
    sSleep.tv_sec = 0;
    sSleep.tv_nsec = 100000000; // .1 sec sleep

    nanosleep(&sSleep, nullptr);

    std::string strFullHost = strHost + ":" + std::to_string(nPort);

    LOG << "Connecting to Vicon Datastream server at " << strFullHost << std::endl;
    LOG.Flush();

    UInt8 unRetries = 10;
    while(unRetries > 0) {
        m_cClient.Connect(strFullHost);
        if (m_cClient.IsConnected().Connected){
            break;
        }
        nanosleep(&sSleep, nullptr);
        unRetries--;
        LOG << "Failed to connect. " << unRetries << "retried remaining" << std::endl;
        LOG.Flush();
    }
    if(!m_cClient.IsConnected().Connected)
        THROW_ARGOSEXCEPTION("Could not reconnect to Vicon Datastream "
                              <<"Server at " << strHost);

    LOG << "Successfully connected." << std::endl;

    m_cClient.SetStreamMode(ViconSDK::StreamMode::ClientPull);
    m_cClient.SetAxisMapping(ViconSDK::Direction::Forward,
                             ViconSDK::Direction::Left,
                             ViconSDK::Direction::Up);
    m_cClient.EnableSegmentData();// nessesary?
    m_cClient.EnableMarkerData(); // nessesary?

    nanosleep(&sSleep, nullptr); // nessesary?
    LOG << "Finished Init tracking engine" << std::endl;
}

TRobotInfoList CViconUpdater::Update(){
    std::string strSubName, strSegName, strName, strType, strRobotOptions;
    UInt16 unSubjectCount, unSegmentCount;
    TRobotInfoList tRobotInfoList;
    m_cClient.GetFrame();

    ViconSDK::Output_GetSubjectCount  cOutput = m_cClient.GetSubjectCount();
    unSubjectCount = cOutput.SubjectCount;

    // LOG <<"There are "<< unSubjectCount <<" subjects." << std::endl;
    for(UInt16 unSubject = 0; unSubject < unSubjectCount; unSubject++)
    {
        strSubName = m_cClient.GetSubjectName(unSubject).SubjectName;
        // LOG << "Robot Type is '" << strType << "' and is a valid type." << std::endl;
        unSegmentCount = m_cClient.GetSegmentCount(strSubName).SegmentCount;

        if(unSegmentCount > 0)
        {
            strSegName = m_cClient.GetSegmentName(strSubName, 0).SegmentName;

            ViconSDK::Output_GetSegmentGlobalTranslation cTrans =
                m_cClient.GetSegmentGlobalTranslation(strSubName, 
                                                       strSegName);


            ViconSDK::Output_GetSegmentGlobalRotationQuaternion cQuat =
                m_cClient.GetSegmentGlobalRotationQuaternion(strSubName,
                                                              strSegName);

            CVector3 cOrigin;
            CQuaternion cRotation;


            if(cTrans.Result == ViconSDK::Result::Success &&
                cQuat.Result == ViconSDK::Result::Success && 
                !cTrans.Occluded && !cQuat.Occluded) 
            {

                cOrigin.Set(
                    cTrans.Translation[0] / 1000.0,
                    cTrans.Translation[1] / 1000.0,
                    cTrans.Translation[2] / 1000.0);

                cRotation.SetW(cQuat.Rotation[3]);
                cRotation.SetX(cQuat.Rotation[0]);
                cRotation.SetY(cQuat.Rotation[1]);
                cRotation.SetZ(cQuat.Rotation[2]);

                GetRobotTypeFromName(strSegName, strType, strName, strRobotOptions);

                TRobotInfo tRobotInfo(strType, strName, cOrigin, cRotation, strRobotOptions);
                tRobotInfoList.push_back(tRobotInfo);
                // LOG << strSubName << " is at:" << cTranslation << std::endl
                //     << " with rotation " << cRotation << std::endl;
            }
        }
    }
    return tRobotInfoList;
}

/****************************************/
/****************************************/

void CViconUpdater::Destroy(){
    LOG << "Disconnecting." << std::endl;
    m_cClient.Disconnect();

    struct timespec sSleep;
    sSleep.tv_sec = 0;
    sSleep.tv_nsec = 100000000;

    while(m_cClient.IsConnected().Connected)
    {
        LOGERR << "Client is still connected, reattempting to disconnect" 
               << std::endl;
        LOGERR.Flush();
        m_cClient.Disconnect();
        nanosleep(&sSleep, nullptr);
    }
}

/****************************************/
/****************************************/

void CViconUpdater::GetRobotTypeFromName(const std::string&  str_original_name,
                                           std::string& str_type,
                                           std::string& str_name,
                                           std::string& str_options){
    if(str_original_name.empty())
        return;
    std::stringstream cStringStream, cNameStringStream;

    cStringStream.str(str_original_name);
    std::getline(cStringStream, str_name, '\'');
    std::getline(cStringStream, str_options);
    
    if(str_name.empty())
        return;
    cNameStringStream.str(str_name);
    std::getline(cNameStringStream, str_type, '_');
    
}

REGISTER_TRACKING_UPDATER(CViconUpdater, "vicon_updater");
