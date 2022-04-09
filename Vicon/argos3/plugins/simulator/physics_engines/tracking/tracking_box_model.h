#ifndef TRACKING_BOX_MODEL_H
#define TRACKING_BOX_MODEL_H

namespace argos {
    class CEmbodiedEntity;
}

#include <argos3/plugins/simulator/physics_engines/tracking/tracking_model.h>

#include <argos3/core/utility/math/box.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

class CTrackingBoxModel : public CTrackingModel {

public:

    CTrackingBoxModel(CTrackingEngine& c_engine,
                      CBoxEntity& c_box_entity);

    void CalculateBoundingBox() override;

    void UpdateFromEntityStatus() override;

    bool CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray) override;

    void SetSize(const CVector3& c_size);

protected:
    CBoxEntity  m_cBoxEntity;
    CBox        m_cShape;
};

#endif
