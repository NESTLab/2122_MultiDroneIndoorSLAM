#include "tracking_cylinder_model.h"


CTrackingCylinderModel::CTrackingCylinderModel(CTrackingEngine& c_engine,
                                               CCylinderEntity& c_cylinder_entity):
    CTrackingModel(c_engine,
                   c_cylinder_entity.GetEmbodiedEntity()),
    m_cShape(c_cylinder_entity.GetRadius(),
             c_cylinder_entity.GetHeight(),
             c_cylinder_entity.GetEmbodiedEntity().GetOriginAnchor().Position,
             CVector3::Z),
    m_cCylinderEntity(c_cylinder_entity){
    CVector3 cAxis;
    CRadians cAngle;
    m_cCylinderEntity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToAngleAxis(cAngle, cAxis);
    m_cPosition = c_cylinder_entity.GetEmbodiedEntity().GetOriginAnchor().Position;
    m_cShape.SetAxis(cAxis);
    m_cShape.SetBasePosition(m_cPosition);
    RegisterAnchorMethod(GetEmbodiedEntity().GetOriginAnchor(),
                         &CTrackingModel::UpdateOriginAnchor);
}

void CTrackingCylinderModel::CalculateBoundingBox(){
    Real fRadius = m_cCylinderEntity.GetRadius();
    // set the corners bounding box
    GetBoundingBox().MinCorner.Set(
        GetEmbodiedEntity().GetOriginAnchor().Position.GetX() - fRadius,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetY() - fRadius,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
    GetBoundingBox().MaxCorner.Set(
        GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + fRadius,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + fRadius,
        GetEmbodiedEntity().GetOriginAnchor().Position.GetZ() + m_cCylinderEntity.GetHeight());
}

void CTrackingCylinderModel::UpdateFromEntityStatus(){
    CVector3 cNewAxis, cOldAxis(m_cShape.GetAxis());
    CRadians cAngle;
    m_cOrientation.ToAngleAxis(cAngle, cNewAxis);
    m_cShape.SetBasePosition(m_cPosition);
    m_cShape.SetAxis(cNewAxis);
}

bool CTrackingCylinderModel::CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray){
    bool bIntersects = m_cShape.Intersects(f_t_on_ray, c_ray);
    return bIntersects;
}

