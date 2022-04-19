#include "tracking_box_model.h"


CTrackingBoxModel::CTrackingBoxModel(CTrackingEngine& c_engine,
                                    CBoxEntity& c_box_entity):
    CTrackingModel(c_engine,
                   c_box_entity.GetEmbodiedEntity()),
    m_cShape(c_box_entity.GetSize(),
             c_box_entity.GetEmbodiedEntity().GetOriginAnchor().Position,
             c_box_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation),
    m_cBoxEntity(c_box_entity){
    m_cPosition = c_box_entity.GetEmbodiedEntity().GetOriginAnchor().Position;
    m_cOrientation = c_box_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation;
    RegisterAnchorMethod(GetEmbodiedEntity().GetOriginAnchor(),
                         &CTrackingModel::UpdateOriginAnchor);
}

void CTrackingBoxModel::CalculateBoundingBox(){
    CVector3 cHalfSize = 0.5f * m_cBoxEntity.GetSize();
    // set the corners of the bounding box
    GetBoundingBox().MinCorner.Set(
        GetEmbodiedEntity().GetOriginAnchor().Position.GetX() - cHalfSize.GetX(),
        GetEmbodiedEntity().GetOriginAnchor().Position.GetY() - cHalfSize.GetY(),
        GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
    GetBoundingBox().MaxCorner.Set(
        GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + cHalfSize.GetX(),
        GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + cHalfSize.GetY(),
        GetEmbodiedEntity().GetOriginAnchor().Position.GetZ() + m_cBoxEntity.GetSize().GetZ());
}

void CTrackingBoxModel::UpdateFromEntityStatus(){
    m_cShape.SetBasePosition(m_cPosition);
    m_cShape.SetOrientation(m_cOrientation);
}

void CTrackingBoxModel::SetSize(const CVector3& c_size){
    // m_cBoxEntity.SetSize(c_size);
    // m_cShape.SetSize(c_size);
}

bool CTrackingBoxModel::CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray){
    bool bIntersects = m_cShape.Intersects(f_t_on_ray, c_ray);

    return bIntersects;
}

