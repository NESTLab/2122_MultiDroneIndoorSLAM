#ifndef KHEPERA_pheromone_LAYER_H
#define KHEPERA_pheromone_LAYER_H

#include <argos3/plugins/controllers/khepera_diffusion/khepera_diffusion.h>
#include <argos3/plugins/robots/generic/control_interface/ci_pheromone_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

using namespace argos;

class CKheperaPheromoneFollower: public CKheperaDiffusion{
public:

    void Init(TConfigurationNode& t_node) override;

    void ControlStep() override;

    void Reset() override {}

    void Destroy() override {}

    CVector2 GoalVector() override;

private:
    CCI_PheromoneSensor* m_pcPheromone;
    CCI_PositioningSensor* m_pcPosition;

};

#endif
