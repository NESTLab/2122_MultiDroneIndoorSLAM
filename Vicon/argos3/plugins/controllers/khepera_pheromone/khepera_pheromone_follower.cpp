#include "khepera_pheromone_follower.h"
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

void CKheperaPheromoneFollower::Init(TConfigurationNode& t_node) {
    CKheperaDiffusion::Init(t_node);
    try {
        m_pcPheromone = GetSensor<CCI_PheromoneSensor>("pheromone");
        m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
    }catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }
}

/****************************************/
/****************************************/

void CKheperaPheromoneFollower::ControlStep() {
    CVector2 cGoalVector = GoalVector();

    if(cGoalVector == CVector2())
        CKheperaDiffusion::ControlStep();
    else
        SetWheelSpeedsFromVector(cGoalVector-ObstacleVector());
}


/****************************************/
/****************************************/

CVector2 CKheperaPheromoneFollower::GoalVector() {
    CVector3 cGradient = m_pcPheromone->GetGradient();
    CRadians cX, cY, cZ;

    if(cGradient == CVector3::ZERO)
        return CVector2{};

    CCI_PositioningSensor::SReading sReadings;
    CVector2 cGoal, cPos, cRotationCenter;

    // Rotate to match robot reference frame
    sReadings = m_pcPosition->GetReading();
    sReadings.Orientation.ToEulerAngles(cZ, cY, cX);

    // LOG << "Orientation:" << sReadings.Orientation << "|cX:" << cX
    //     << "|cY:" << cY  <<"|cZ:" << cZ << std::endl;
    //convert to 2d
    cGradient.ProjectOntoXY(cGoal);
    sReadings.Position.ProjectOntoXY(cPos);

    return cGoal.Rotate(-cZ);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CKheperaPheromoneFollower, "khepera_pheromone_follower_controller")
