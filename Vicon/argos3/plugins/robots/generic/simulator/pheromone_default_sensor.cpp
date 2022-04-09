#include <string>
#include <map>

#include "pheromone_default_sensor.h"
#include <argos3/core/simulator/simulator.h>

/****************************************/
/****************************************/

void CPheromoneSensor::SetRobot(CComposableEntity& c_entity){
    m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
    // set initial readings
}

/****************************************/
/****************************************/

void CPheromoneSensor::Init(TConfigurationNode& t_tree){
    m_fSize = .5;
    try {
       /* Parent class init */
       CCI_PheromoneSensor::Init(t_tree);
       /* Get pheromone medium from id specified in the XML */
       std::string strMedium;
       GetNodeAttribute(t_tree, "medium", strMedium);
       GetNodeAttributeOrDefault(t_tree, "range", m_fSize, m_fSize);
       m_pCPheromoneMedium = &(CSimulator::GetInstance().GetMedium<CPheromoneMedium>(strMedium));
    }
    catch(CARGoSException& ex) {
       THROW_ARGOSEXCEPTION_NESTED("Error initializing the range and bearing medium sensor", ex);
    }
}

/****************************************/
/****************************************/

void CPheromoneSensor::Update(){
    CVector3 cPosition = m_pcEmbodiedEntity->GetOriginAnchor().Position;
    m_tReadings = m_pCPheromoneMedium->ReadPheromone(cPosition, m_fSize);
    Real fPheroRes = m_pCPheromoneMedium->GetCellSize();
    UInt16 unLargestReading = 0;

    for(const auto& tReading: m_tReadings){
        if(tReading.second > unLargestReading){
            unLargestReading = tReading.second;
            m_cGradient = tReading.first;
        }
    }

    if(unLargestReading == 0)
        m_cGradient = CVector3::ZERO;
    else{
        Real cZOffset = m_pCPheromoneMedium->IsThreeDimensions()? fPheroRes/2:0;
        //
        m_cGradient += CVector3(fPheroRes/2, fPheroRes/2, cZOffset);
    }
}

/****************************************/
/****************************************/

void CPheromoneSensor::Reset(){
    m_tReadings.clear();
    m_cGradient = CVector3::ZERO;
}

/****************************************/
/****************************************/

void CPheromoneSensor::Destroy(){

}

/****************************************/
/****************************************/

REGISTER_SENSOR(CPheromoneSensor,
                "pheromone", "default",
                "Chris Cormier [ccormier@wpi.com]",
                "0.1",
                "A generic pheromone sensor.",
                "This sensor allows robots to perform read pheromone in the surrounding area.\n"
                "The sensor can also determine the direction of the strongest pheromone \n"
                "in the surrounding area.\n"
                "This implementation of the pheromone sensor is associated to the\n"
                "pheromone medium. To be able to use this sensor, you must add a\n"
                "pheromone medium to the <media> section.\n"
                "This sensor allows a robot to read a pheromone trail. To lay pheromone, you \n"
                "need the pheromone actuator.\n"
                "To use this sensor, in controllers you must include the\n"
                "ci_pheromone_sensor.h header.\n\n"
                "REQUIRED XML CONFIGURATION\n\n"
                "  <controllers>\n"
                "    ...\n"
                "    <my_controller ...>\n"
                "      ...\n"
                "      <sensors>\n"
                "        ...\n"
                "        <pheromone implementation=\"default\"\n"
                "                           medium=\"pheromone\" />\n"
                "        ...\n"
                "      </sensors>\n"
                "      ...\n"
                "    </my_controller>\n"
                "    ...\n"
                "  </controllers>\n\n"
                "The 'medium' attribute must be set to the id of the pheromone medium\n"
                "declared in the <media> section.\n\n"
                "OPTIONAL XML CONFIGURATION\n\n"
                "It is possible to specify the size of the area in which the pheromone\n"
                "is read. The area size is defined in meters as the maximum deviation in \n"
                "any coordinate. This produces a square or cube area, circular and \n"
                "spherical areas are planned for future release.\n\n"
                "The default range is 0.5, to use a different range add the range \n"
                "tag as done in this example:\n"
                "  <controllers>\n"
                "    ...\n"
                "    <my_controller ...>\n"
                "      ...\n"
                "      <sensors>\n"
                "        ...\n"
                "        <pheromone implementation=\"medium\"\n"
                "                           medium=\"pheromone\" />\n"
                "                           range=1.5 />\n"
                "        ...\n"
                "      </sensors>\n"
                "      ...\n"
                "    </my_controller>\n"
                "    ...\n"
                "  </controllers>\n\n",
                "In Development"
)