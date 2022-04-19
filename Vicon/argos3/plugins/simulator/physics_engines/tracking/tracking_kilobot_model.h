#ifndef TRACKING_KILOBOT_MODEL_H
#define TRACKING_KILOBOT_MODEL_H

namespace argos {
    class CEmbodiedEntity;
}

#include <argos3/plugins/simulator/physics_engines/tracking/tracking_model.h>
#include <argos3/core/utility/math/cylinder.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>

class CTrackingKilobotModel : public CTrackingModel {

public:

    CTrackingKilobotModel(CTrackingEngine& c_engine,
                         CKilobotEntity& c_f_entity);

    virtual ~CTrackingKilobotModel(){}

    virtual void CalculateBoundingBox();

    virtual void UpdateFromEntityStatus();

    virtual void UpdateEntityStatus();

    bool CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray);

protected:
    CCylinder        m_cShape;

};

#endif
