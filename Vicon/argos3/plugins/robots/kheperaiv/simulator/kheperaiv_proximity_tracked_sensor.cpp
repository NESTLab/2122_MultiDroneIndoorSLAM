#include "kheperaiv_proximity_tracked_sensor.h"

/****************************************/
/****************************************/

void CKheperaIVProximityTrackedSensor::SetRobot(CComposableEntity& c_entity) {
    m_pcEntity = &c_entity;
}

/****************************************/
/****************************************/

void CKheperaIVProximityTrackedSensor::Init(TConfigurationNode& t_tree) {
}

/****************************************/
/****************************************/

void CKheperaIVProximityTrackedSensor::Update() {
    if(!m_bInitialized){
        CEmbodiedEntity *cBody = &(m_pcEntity->GetComponent<CEmbodiedEntity>("body"));
        m_pcModel = dynamic_cast<CTrackingKheperaModel* >(&(cBody->GetPhysicsModel("tracking_engine")));
        if(m_pcModel == nullptr)
            THROW_ARGOSEXCEPTION("Can't set robot for the Khepera IV proximity tracked sensor");
        m_pcModel->ReserveSensor("IRSensorDataRaw", 10);
        m_bInitialized = true;

    }else{
        CByteArray* cIRSensorData = m_pcModel->GetSensorData("IRSensorDataRaw");
        UInt16 cTempValues[8];
        //ammount of bits perSensor
        UInt8 unSensorDataSize = 10;
        // unused bits in current byte from the buffer
        UInt8 unBufferBitIndex = 8;
        // bits still needed for sensor data
        UInt8 unDataBitIndex = unSensorDataSize;

        // Zero the array
        for(auto& cTempValue: cTempValues){
            cTempValue = 0;
        }
        auto unWorkingByte = cIRSensorData->PopFront<UInt8>();
        UInt8 unCurrentSensor = 0;
        SInt8 snBitDifference;

        // loop through all sensors
        while(unCurrentSensor < 8){
            snBitDifference = unBufferBitIndex - unDataBitIndex;
            // all the needed bits are in the working byte
            if(snBitDifference >= 0){
                cTempValues[unCurrentSensor] |=  unWorkingByte >> snBitDifference;

                // we are done with this sensor, keep only wanted bits,
                // reset data index and use the next sensor
                cTempValues[unCurrentSensor] &= 0x3ff;
                unDataBitIndex = unSensorDataSize;
                unCurrentSensor += 1;

                // if we are done with the byte from the buffer 
                // and there are more sensors to fill
                // reset buffer index and grab next byte
                if (snBitDifference == 0 && unCurrentSensor < 8){
                    unBufferBitIndex = 8;
                    unWorkingByte = cIRSensorData->PopFront<UInt8>();
                } else
                    unBufferBitIndex = snBitDifference;
            }else{
                // only some of the needed bits are 
                cTempValues[unCurrentSensor] |= unWorkingByte << -snBitDifference;
                
                // we are done with this byte, reset index and grabe next byte
                unBufferBitIndex = 8;
                unWorkingByte = cIRSensorData->PopFront<UInt8>();

                // decrease the ammount of needed bytes for this sensor
                unDataBitIndex = -snBitDifference;
            }
        }

        // 
        // >fMin is too far, <fMax is too close
        // LOG << "Sensor Readings for " 
        //     <<  m_pcModel->GetEmbodiedEntity().GetContext() << " "; 
        Real fMax = 1000.0f;
        Real fMinCutoff = 150.0f;
        Real fMin = 100.0f;
        for(UInt8 i = 0; i < 8; i++){
            if(cTempValues[i] < fMinCutoff)
                m_tReadings[(11-i)%8].Value = 0.0f;
            else if(cTempValues[i] > fMax)
                m_tReadings[(11-i)%8].Value = 1.0f;
            else{
                m_tReadings[(11-i)%8].Value = ((Real)cTempValues[i]-fMin)/(fMax-fMin) ;
            }
            
            // LOG << "(" << cTempValues[i] << ", "
            //     << m_tReadings[(11-i)%8].Value << ") ";
        }
        // LOG << std::endl;
        free(cIRSensorData);
    }
}

/****************************************/
/****************************************/

void CKheperaIVProximityTrackedSensor::Reset() {

}

/****************************************/
/****************************************/

REGISTER_SENSOR(CKheperaIVProximityTrackedSensor,
 "kheperaiv_proximity", "tracked",
 "Chris Cormier [ccormier@wpi.edu]",
 "0.1",
 "The Khepera IV proximity sensor.",
 "This sensor accesses the Khepera IV proximity sensor. For a complete description\n"
 "of its usage, refer to the ci_kheperaiv_proximity_sensor.h interface.\n"
 "REQUIRED XML CONFIGURATION\n\n"
 "  <controllers>\n"
 "    ...\n"
 "    <my_controller ...>\n"
 "      ...\n"
 "      <sensors>\n"
 "        ...\n"
 "        <proximity implementation=\"tracked\" />\n"
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
 "        <proximity implementation=\"tracked\"\n"
 "                   show_rays=\"true\" />\n"
 "        ...\n"
 "      </sensors>\n"
 "      ...\n"
 "    </my_controller>\n"
 "    ...\n"
 "  </controllers>\n\n",
 "Usable"
 );