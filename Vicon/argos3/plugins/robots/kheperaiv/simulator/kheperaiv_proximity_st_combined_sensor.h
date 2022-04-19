#ifndef KHEPERAIV_PROXIMITY_ST_COMBINED_SENSOR_H
#define KHEPERAIV_PROXIMITY_ST_COMBINED_SENSOR_H

#include <string>
#include <map>

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>
#include <argos3/plugins/robots/generic/simulator/proximity_default_sensor.h>
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_khepera_model.h>

#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_proximity_default_sensor.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_proximity_tracked_sensor.h>

class CKheperaIVProximitySTCombinedSensor : public CCI_KheperaIVProximitySensor,
                                            public CSimulatedSensor {

public:

    CKheperaIVProximitySTCombinedSensor();

    ~CKheperaIVProximitySTCombinedSensor() override;
    
    void SetRobot(CComposableEntity& c_entity) override;

    void Init(TConfigurationNode& t_tree) override;

    void Update() override;

    void Reset() override;

private:
    CKheperaIVProximityTrackedSensor* m_pcTracked;
    CKheperaIVProximityDefaultSensor* m_pcDefault;

    bool m_bShowRays;
    bool m_bAddNoise = false;
    CRange<Real> m_cNoiseRange;
    CRandom::CRNG* m_pcRNG;

    /** Reference to proximity sensor equipped entity associated to this sensor */
    CProximitySensorEquippedEntity* m_pcProximityEntity;

    /** Reference to controllable entity associated to this sensor */
    CControllableEntity* m_pcControllableEntity;
};

#endif