
#ifndef PHEROMONE_MEDIUM_SENSOR_H
#define PHEROMONE_MEDIUM_SENSOR_H

#include <string>
#include <map>

#include <argos3/core/simulator/actuator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_pheromone_actuator.h>
#include <argos3/plugins/simulator/media/pheromone_medium.h>

using namespace argos;

class CPheromoneActuator : public CSimulatedActuator,
                           public CCI_PheromoneActuator {

public:
   void SetRobot(CComposableEntity& c_entity) override;
   void Init(TConfigurationNode& t_tree) override;
   void Update() override;
   void Reset() override;
   void Destroy() override;

private:

   /** Reference to embodied entity associated with this actuator */
   CEmbodiedEntity*     m_pcEmbodiedEntity;
   /** Reference to pheromone medium associated with this actuator */
   CPheromoneMedium*    m_pCPheromoneMedium;

};

#endif
