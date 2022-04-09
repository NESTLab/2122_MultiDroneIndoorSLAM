#include "kheperaiv_proximity_st_combined_sensor.h"

/****************************************/
/****************************************/

CKheperaIVProximitySTCombinedSensor::CKheperaIVProximitySTCombinedSensor():
    m_pcTracked(new CKheperaIVProximityTrackedSensor()),
    m_pcDefault(new CKheperaIVProximityDefaultSensor()) {}

/****************************************/
/****************************************/

CKheperaIVProximitySTCombinedSensor::~CKheperaIVProximitySTCombinedSensor(){
    free(m_pcTracked);
    free(m_pcDefault);
}

/****************************************/
/****************************************/

void CKheperaIVProximitySTCombinedSensor::SetRobot(CComposableEntity& c_entity) {
    m_pcTracked->SetRobot(c_entity);
    m_pcDefault->SetRobot(c_entity);

    try {
        m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
        m_pcProximityEntity = &(c_entity.GetComponent<CProximitySensorEquippedEntity>("proximity_sensors"));
        // m_pcProximityEntity->Enable();
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the proximity default sensor", ex);
    }
}

/****************************************/
/****************************************/

void CKheperaIVProximitySTCombinedSensor::Init(TConfigurationNode& t_tree) {
    TConfigurationNode tEmptyNode;
    // init with empty node
    m_pcTracked->Init(tEmptyNode);
    m_pcDefault->Init(tEmptyNode);

    m_bShowRays = false;
    GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
    /* Parse noise level */
    Real fNoiseLevel = 0.0f;
    GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
    if(fNoiseLevel < 0.0f) {
        THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the proximity sensor");
    }
    else if(fNoiseLevel > 0.0f) {
        m_bAddNoise = true;
        m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
        m_pcRNG = CRandom::CreateRNG("argos");
    }
}
    
/****************************************/
/****************************************/

void CKheperaIVProximitySTCombinedSensor::Update() {
    m_pcTracked->Update();
    m_pcDefault->Update();

    TReadings cTrackedReadings = m_pcTracked->GetReadings();
    TReadings cDefaultReadings = m_pcDefault->GetReadings();

    for(size_t i = 0; i < cTrackedReadings.size(); i++){
        SReading sCurrDefault = cDefaultReadings[i];
        // add noise on a simulated sensor
        if(m_bAddNoise)
            sCurrDefault.Value += m_pcRNG->Uniform(m_cNoiseRange);

        //keep reading with larger value
        m_tReadings[i] = std::max(sCurrDefault, cTrackedReadings[i],
                                  [](const SReading& a, const SReading& b){
                                    return a.Value < b.Value;
                                  });

        // check to draw the rays
        if(m_bShowRays){
            CRay3 cScanningRay;
            CVector3 cRayStart, cRayEnd;
            /* Compute ray for sensor i */
            cRayStart = m_pcProximityEntity->GetSensor(i).Offset;
            cRayStart.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
            cRayStart += m_pcProximityEntity->GetSensor(i).Anchor.Position;

            cRayEnd = m_pcProximityEntity->GetSensor(i).Offset;
            cRayEnd += m_pcProximityEntity->GetSensor(i).Direction;
            cRayEnd.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
            cRayEnd += m_pcProximityEntity->GetSensor(i).Anchor.Position;
            cScanningRay.Set(cRayStart,cRayEnd);

            // compute the value
            Real fLength = log((m_tReadings[i].Value+0.085)/4.14)/(-33.0)/cScanningRay.GetLength();
            if(m_tReadings[i].Value == 0){
                // no intersection
                m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
            }else{
                // intersection
                m_pcControllableEntity->AddIntersectionPoint(cScanningRay,fLength);
                m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
            }
        }
    }
}

/****************************************/
/****************************************/

void CKheperaIVProximitySTCombinedSensor::Reset() {
    m_pcTracked->Reset();
    m_pcDefault->Reset();
}   

/****************************************/
/****************************************/

REGISTER_SENSOR(CKheperaIVProximitySTCombinedSensor,
 "kheperaiv_proximity", "st_combined",
 "Chris Cormier [ccormier@wpi.edu]",
 "0.1",
 "The Khepera IV proximity sensor.",
 "This sensor combines the the default and tracked sensor.\n"
 "REQUIRED XML CONFIGURATION\n\n"
 "  <controllers>\n"
 "    ...\n"
 "    <my_controller ...>\n"
 "      ...\n"
 "      <sensors>\n"
 "        ...\n"
 "        <proximity implementation=\"st_combined\" />\n"
 "        ...\n"
 "      </sensors>\n"
 "      ...\n"
 "    </my_controller>\n"
 "    ...\n"
 "  </controllers>\n\n"
 "OPTIONAL XML CONFIGURATION\n\n"
 "It is possible to draw the rays shot by the proximity sensor in the OpenGL\n"
 "visualization. This can be useful for sensor debugging but also to understand\n"
 "what's wrong in your controller. In OpenGL, the rays are drawn in cyan when\n"
 "they are not obstructed and in purple when they are. In case a ray is\n"
 "obstructed, a black dot is drawn where the intersection occurred.\n"
 "To turn this functionality on, add the attribute \"show_rays\" as in this\n"
 "example:\n\n"
 "  <controllers>\n"
 "    ...\n"
 "    <my_controller ...>\n"
 "      ...\n"
 "      <sensors>\n"
 "        ...\n"
 "        <proximity implementation=\"st_combined\"\n"
 "                   show_rays=\"true\" />\n"
 "        ...\n"
 "      </sensors>\n"
 "      ...\n"
 "    </my_controller>\n"
 "    ...\n"
 "  </controllers>\n\n",
 "Usable"
 );