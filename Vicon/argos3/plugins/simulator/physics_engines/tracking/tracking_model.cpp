#include "tracking_model.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_engine.h>

/****************************************/
/****************************************/

CTrackingModel::CTrackingModel(CTrackingEngine& c_engine,
                               CEmbodiedEntity& c_entity) :
   CPhysicsModel(c_engine,
                 c_entity){
    /* Register the origin anchor update method */
    RegisterAnchorMethod(GetEmbodiedEntity().GetOriginAnchor(),
                         &CTrackingModel::UpdateOriginAnchor);
    c_entity.AddPhysicsModel(c_engine.GetId(), *this);
    m_bConnected = false;
    m_bTracked = false;
}

/****************************************/
/****************************************/

void CTrackingModel::UpdateFromEntityStatus() {
}

/****************************************/
/****************************************/

void CTrackingModel::UpdateEntityStatus() {
    CPhysicsModel::UpdateEntityStatus();
}

/****************************************/
/****************************************/

void CTrackingModel::MoveTo(const CVector3& c_position,
                            const CQuaternion& c_orientation) {
}


bool CTrackingModel::CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray){
    return false;
}

/****************************************/
/****************************************/

bool CTrackingModel::IsCollidingWithSomething() const{
   return false;
}

/****************************************/
/****************************************/

void CTrackingModel::UpdateOriginAnchor(SAnchor& s_anchor) {
    s_anchor.Position = m_cPosition;
    s_anchor.Orientation = m_cOrientation;
}

/****************************************/
/****************************************/

CVector3 CTrackingModel::GetPosition(){
    return m_cPosition;
}

/****************************************/
/****************************************/

CQuaternion CTrackingModel::GetOrientation(){
    return m_cOrientation;
}

/****************************************/
/****************************************/

void CTrackingModel::SetPosition(CVector3 c_position){
    m_cPosition = c_position;
}

/****************************************/
/****************************************/

void CTrackingModel::SetOrientation(CQuaternion c_orientation){
    m_cOrientation = c_orientation;
}

/****************************************/
/****************************************/

void CTrackingModel::SetConnection(CMTCPSocket c_connection){
    m_cConnection = std::move(c_connection);
}

/****************************************/
/****************************************/

void CTrackingModel::SetTracked(){
    m_bTracked = true;
}

/****************************************/
/****************************************/

void CTrackingModel::SetConnected(){
    m_bConnected = true;
}

/****************************************/
/****************************************/

bool CTrackingModel::IsTracked(){
    return m_bTracked;
}


/****************************************/
/****************************************/
bool CTrackingModel::IsConnected(){
    return m_bConnected;
}
