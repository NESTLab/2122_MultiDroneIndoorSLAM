#ifndef KHEPERA_PHEROMONE_LAYER_H
#define KHEPERA_PHEROMONE_LAYER_H

#include <argos3/plugins/controllers/khepera_diffusion/khepera_diffusion.h>
#include <argos3/plugins/robots/generic/control_interface/ci_pheromone_actuator.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CKheperaPheromoneLayer : public CKheperaDiffusion {

public:
   void Init(TConfigurationNode& t_node) override;

   void ControlStep() override;

   void Reset() override {}

   void Destroy() override {}

private:

   CCI_PheromoneActuator* m_pcPheromone = nullptr;
};

#endif
