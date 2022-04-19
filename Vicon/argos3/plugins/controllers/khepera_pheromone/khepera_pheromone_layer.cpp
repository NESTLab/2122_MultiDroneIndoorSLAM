#include "khepera_pheromone_layer.h"

/****************************************/
/****************************************/

void CKheperaPheromoneLayer::Init(TConfigurationNode& t_node) {
    CKheperaDiffusion::Init(t_node);
    m_pcPheromone = GetActuator<CCI_PheromoneActuator>("pheromone");
}

/****************************************/
/****************************************/

void CKheperaPheromoneLayer::ControlStep() {
    // CKheperaDiffusion::ControlStep();
    SetWheelSpeedsFromVector(GoalVector()-ObstacleVector(), true);
    m_pcPheromone->SetLaying(true);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CKheperaPheromoneLayer, "khepera_pheromone_layer_controller")
