#ifndef TRACKING_CYLINDER_MODEL_H
#define TRACKING_CYLINDER_MODEL_H

namespace argos {
    class CEmbodiedEntity;
}

#include <argos3/plugins/simulator/physics_engines/tracking/tracking_model.h>

#include <argos3/core/utility/math/cylinder.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>

class CTrackingCylinderModel : public CTrackingModel {

public:

    CTrackingCylinderModel(CTrackingEngine& c_engine,
                      CCylinderEntity& c_cylinder_entity);

    void CalculateBoundingBox() override;

    void UpdateFromEntityStatus() override;

    bool CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray);

protected:
    CCylinderEntity m_cCylinderEntity;
    CCylinder       m_cShape;
};

#endif
