#include "tracking_kilobot_model.h"

/****************************************/
/****************************************/

static Real KILOBOT_RADIUS = 0.015;
static Real KILOBOT_HEIGHT = 0.055;

/****************************************/
/****************************************/

CTrackingKilobotModel::CTrackingKilobotModel(CTrackingEngine& c_engine,
                                             CKilobotEntity& c_f_entity) :
    CTrackingModel(c_engine,
                   c_f_entity.GetEmbodiedEntity()),
    m_cShape(KILOBOT_RADIUS,
             KILOBOT_HEIGHT,
             c_f_entity.GetEmbodiedEntity().GetOriginAnchor().Position)
{
    RegisterAnchorMethod(GetEmbodiedEntity().GetOriginAnchor(),
                         &CTrackingModel::UpdateOriginAnchor);
    
}

/****************************************/
/****************************************/

void CTrackingKilobotModel::CalculateBoundingBox(){
    GetBoundingBox().MinCorner.Set(
        GetEmbodiedEntity().GetOriginAnchor().Position.GetX() - KILOBOT_RADIUS,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetY() - KILOBOT_RADIUS,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
    GetBoundingBox().MaxCorner.Set(
        GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + KILOBOT_RADIUS,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + KILOBOT_RADIUS,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetZ() + KILOBOT_HEIGHT);
}

void CTrackingKilobotModel::UpdateFromEntityStatus(){
    CTrackingModel::UpdateFromEntityStatus();
}

/****************************************/
/****************************************/

bool CTrackingKilobotModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                     const CRay3& c_ray){
    bool bIntersects = m_cShape.Intersects(f_t_on_ray, c_ray);
    return bIntersects;
}

/****************************************/
/****************************************/

void CTrackingKilobotModel::UpdateEntityStatus(){
    CTrackingModel::UpdateEntityStatus();
    // New Axis should be Z rotated by orientation
    CVector3 cNewAxis(CVector3::Z);
    cNewAxis.Rotate(m_cOrientation);
    m_cShape.SetBasePosition(m_cPosition);
    m_cShape.SetAxis(cNewAxis);
}

/****************************************/
/****************************************/
