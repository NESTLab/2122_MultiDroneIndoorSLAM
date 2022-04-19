#ifndef KHEPERAIV_PROXIMITY_TRACKED_SENSOR_H
#define KHEPERAIV_PROXIMITY_TRACKED_SENSOR_H

#include <string>
#include <map>

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>
#include <argos3/plugins/robots/generic/simulator/proximity_default_sensor.h>
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_khepera_model.h>


class CKheperaIVProximityTrackedSensor : public CCI_KheperaIVProximitySensor,
                                         public CSimulatedSensor {

public:

    void SetRobot(CComposableEntity& c_entity) override;

    void Init(TConfigurationNode& t_tree) override;

    void Update() override;

    void Reset() override;

private:
    CTrackingKheperaModel* m_pcModel = nullptr;
    CComposableEntity* m_pcEntity = nullptr;
    bool m_bInitialized = false;

};

#endif