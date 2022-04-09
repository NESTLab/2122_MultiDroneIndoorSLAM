
#ifndef PHEROMONE_MEDIUM_SENSOR_H
#define PHEROMONE_MEDIUM_SENSOR_H

#include <string>
#include <map>

#include <argos3/core/simulator/sensor.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_pheromone_sensor.h>
#include <argos3/plugins/simulator/media/pheromone_medium.h>

using namespace argos;

class CPheromoneSensor : public CSimulatedSensor,
                         public CCI_PheromoneSensor {

public:
   void SetRobot(CComposableEntity& c_entity) override;
   void Init(TConfigurationNode& t_tree) override;
   void Update() override;
   void Reset() override;
   void Destroy() override;

private:

   /** Reference to embodied entity associated with this sensor */
   CEmbodiedEntity*     m_pcEmbodiedEntity;
   /** Reference to pheromone medium associated with this sensor */
   CPheromoneMedium*    m_pCPheromoneMedium;
   
   Real m_fSize;
};

#endif
