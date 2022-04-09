#include "tracking_khepera_model.h"

/****************************************/
/****************************************/

static Real KHEPERA_RADIUS = .07;
static Real KHEPERA_HEIGHT = 0.175;

/****************************************/
/****************************************/

enum Khepera_WHEELS {
  KHEPERA_LEFT_WHEEL = 0,
  KHEPERA_RIGHT_WHEEL = 1
};

CTrackingKheperaModel::CTrackingKheperaModel(CTrackingEngine& c_engine,
                                             CKheperaIVEntity& c_f_entity) :
    CTrackingModel(c_engine,
                   c_f_entity.GetEmbodiedEntity()),
    m_cShape(KHEPERA_RADIUS,
             KHEPERA_HEIGHT,
             c_f_entity.GetEmbodiedEntity().GetOriginAnchor().Position),
    m_cKheperaEntity(c_f_entity),
    m_cWheeledEntity(m_cKheperaEntity.GetWheeledEntity()),
    m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()),
    m_bFirstPass(false)
{
    RegisterAnchorMethod(GetEmbodiedEntity().GetOriginAnchor(),
                         &CTrackingModel::UpdateOriginAnchor);
    m_tThreadInfo.cConnection = &m_cConnection;
    m_tThreadInfo.run = true;
    m_cSendThread = std::thread(ThreadCallback, &m_tThreadInfo);

    for(int i = 0; i < 4; i++)
        m_tRealCliffDetect[i] = false;
    
}

/****************************************/
/****************************************/

CTrackingKheperaModel::~CTrackingKheperaModel() {
    m_tThreadInfo.run = false;
    m_cSendThread.join();
}


void CTrackingKheperaModel::CalculateBoundingBox(){
    GetBoundingBox().MinCorner.Set(
        GetEmbodiedEntity().GetOriginAnchor().Position.GetX() - KHEPERA_RADIUS,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetY() - KHEPERA_RADIUS,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
    GetBoundingBox().MaxCorner.Set(
        GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + KHEPERA_RADIUS,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + KHEPERA_RADIUS,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetZ() + KHEPERA_HEIGHT);
}

void CTrackingKheperaModel::UpdateFromEntityStatus(){
    CTrackingModel::UpdateFromEntityStatus();
    
    std::vector<CByteArray> cReadMessages;
    std::unique_lock<std::mutex> lock(m_tThreadInfo.cMessagesMutex, std::defer_lock);

    // copy the messages
    lock.lock();
    if(!m_tThreadInfo.cReadMessages.empty()){
        cReadMessages = m_tThreadInfo.cReadMessages;
        m_tThreadInfo.cReadMessages.clear();
    }
    lock.unlock();

    // proccess each message
    while(!cReadMessages.empty()){
        CByteArray cMessage = cReadMessages.back();
        cReadMessages.pop_back();
        auto unMessageType = cMessage.PopFront<UInt8>();
        switch(unMessageType){
            case 0: {
                LOG << "Name:" << cMessage << std::endl;
                break;}
            case 1: {
                LOG << "Disconnection" << std::endl;
                break;}
            case 2: {
                LOG << "Left Speed:" << cMessage[0] << ", Right Speed:" << cMessage[1] << std::endl;
                break;}
            case 3:{
                // proccess a sensor response
                UInt16 unSensorByteMask  = cMessage.PopFront<UInt8>();
                CByteArray cSensorData; 

                if(unSensorByteMask & 1<<7){
                    cSensorData.Resize(5, 0);
                    for(int i = 0; i < 5; i++){
                        cSensorData[i] = cMessage.PopFront<UInt8>();
                    }
                    m_tSensorData.Write("IRSensorData", cSensorData);
                }

                if(unSensorByteMask & 1<<6){
                    cSensorData.Resize(10, 0);
                    for(int i = 0; i < 10; i++){
                        cSensorData[i] = cMessage.PopFront<UInt8>();
                    }
                    m_tSensorData.Write("IRSensorDataRaw", cSensorData);
                }

                if(unSensorByteMask & 1<<5){
                    cSensorData.Resize(5, 0);
                    for(int i = 0; i < 5; i++){
                        cSensorData[i] = cMessage.PopFront<UInt8>();
                    }
                    m_tSensorData.Write("GroundSensors", cSensorData);
                }

                if(unSensorByteMask & 1<<4){
                    cSensorData.Resize(12, 0);
                    for(int i = 0; i < 12; i++){
                        cSensorData[i] = cMessage.PopFront<UInt8>();
                    }
                    m_tSensorData.Write("BatterySensor", cSensorData);

                    // For each field khepera4_test.c line 1762
                }
                break;
            }
            default:
                LOG << "Invalid Message type " << unMessageType << std::endl;
        }
    }
}

/****************************************/
/****************************************/

bool CTrackingKheperaModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                     const CRay3& c_ray){
    bool bIntersects = m_cShape.Intersects(f_t_on_ray, c_ray);
    return bIntersects;
}

/****************************************/
/****************************************/

void CTrackingKheperaModel::UpdateEntityStatus(){
    CTrackingModel::UpdateEntityStatus();

    // update the position and orientation of the robot 
    CVector3 cNewAxis(CVector3::Z);
    // New Axis should be Z rotated by orientation
    cNewAxis.Rotate(m_cOrientation);
    m_cShape.SetBasePosition(m_cPosition);
    m_cShape.SetAxis(cNewAxis);
    
    if(!m_cConnection.IsConnected())
        return;


    CByteArray driveMessage(1);
    CByteArray cSensorRequest(3);
    bool bSendSensorRequest = false;


    // generate a drive message
    UInt8 left_speed = UInt8(m_fCurrentWheelVelocity[KHEPERA_LEFT_WHEEL]*127)*10;
    UInt8 right_speed = UInt8(m_fCurrentWheelVelocity[KHEPERA_RIGHT_WHEEL]*127)*10;
    driveMessage[0] = 4;
    driveMessage << left_speed;
    driveMessage << right_speed;

    // generate a sensor response
    cSensorRequest[0] = 6;
    cSensorRequest[1] = 0;
    if(m_tSensorData.IsReserved("IRSensorData")){
        cSensorRequest[1] |= 1<<7;
        bSendSensorRequest = true;
    }

    if(m_tSensorData.IsReserved("IRSensorDataRaw")){
        cSensorRequest[1] |= 1<<6;
        bSendSensorRequest = true;
    }

    if(m_tSensorData.IsReserved("GroundSensors")){
        cSensorRequest[1] |= 1<<5;
        bSendSensorRequest = true;
    }

    if(m_tSensorData.IsReserved("BatterySensor")){
        cSensorRequest[1] |= 1<<4;
        bSendSensorRequest = true;
    }
    std::lock_guard<std::mutex> lock(m_tThreadInfo.cMessagesMutex);
    m_tThreadInfo.cSendMessages.push_back(driveMessage);
    if(bSendSensorRequest){
        m_tThreadInfo.cSendMessages.push_back(cSensorRequest);
    }
    // CTrackingModel::UpdateEntityStatus();
}

/****************************************/
/****************************************/

void CTrackingKheperaModel::ThreadCallback(TThreadInfo* t_thread_info){
    LOG.AddThreadSafeBuffer();
    LOGERR.AddThreadSafeBuffer();
    std::unique_lock<std::mutex> lock(t_thread_info->cMessagesMutex, std::defer_lock);
    std::vector<CByteArray> cSendMessages;
    CByteArray cReadMessage;

    while (t_thread_info->run){
        if(!t_thread_info->cSendMessages.empty()){
            lock.lock();
            cSendMessages = t_thread_info->cSendMessages;
            t_thread_info->cSendMessages.clear();
            lock.unlock();
        }

        // send all messages in send queue
        while(!cSendMessages.empty()){
            CByteArray cMessage = cSendMessages.back();
            cSendMessages.pop_back();
            t_thread_info->cConnection->SendByteArray(cMessage);
        }

        // receive a message if there is one availible
        // TODO: receive all messages?
        if(t_thread_info->cConnection->IsConnected() &&
           t_thread_info->cConnection->MessageAvalible()){
            if(t_thread_info->cConnection->ReceiveByteArray(cReadMessage)){
                lock.lock();
                t_thread_info->cReadMessages.push_back(cReadMessage);
                lock.unlock();
            }
        }
    }
}

/****************************************/
/****************************************/
