#include "kheperaiv_battery_tracked_sensor.h"

/****************************************/
/****************************************/

void CKheperaIVBatteryTrackedSensor::SetRobot(CComposableEntity& c_entity) {
    m_pcEntity = &c_entity;
}

/****************************************/
/****************************************/

void CKheperaIVBatteryTrackedSensor::Init(TConfigurationNode& t_tree) {
}

/****************************************/
/****************************************/

void CKheperaIVBatteryTrackedSensor::Update() {
    if(!m_bInitialized){
        CEmbodiedEntity *cBody = &(m_pcEntity->GetComponent<CEmbodiedEntity>("body"));
        m_pcModel = dynamic_cast<CTrackingKheperaModel* >(&(cBody->GetPhysicsModel("tracking_engine")));
        if(m_pcModel == nullptr)
            THROW_ARGOSEXCEPTION("Can't set robot for the Khepera IV proximity tracked sensor");
        m_pcModel->ReserveSensor("BatterySensor", 12);
        m_bInitialized = true;

    }else{
        // conversions can be found in khepera4_test.c at line 1762
        CByteArray* pcData = m_pcModel->GetSensorData("BatterySensor");
        // current and verage current in mA
        Real fCurrent = (short)((*pcData)[4] | (*pcData)[5]<<8)*0.07813;
        Real fAverageCurrent = (short)((*pcData)[6] | (*pcData)[7]<<8)*0.07813;
        // remaining capacity in mAh
        UInt16 unCapacity = ((*pcData)[1] | (*pcData)[2]<<8)*1.6;
        // remaining capacity in percent
        m_sReading.AvailableCharge = (*pcData)[3]/100.0;
        m_sReading.TimeLeft = -(unCapacity/fCurrent*3600);

        LOG << "AvailableCharge: " << m_sReading.AvailableCharge << std::endl;
        LOG << "TimeLeft: " << m_sReading.TimeLeft << std::endl;
    }
}

/****************************************/
/****************************************/

void CKheperaIVBatteryTrackedSensor::Reset() {

}

/****************************************/
/****************************************/

REGISTER_SENSOR(CKheperaIVBatteryTrackedSensor,
 "kheperaiv_battery", "tracked",
 "Chris Cormier [ccormier@wpi.edu]",
 "0.1",
 "The Khepera IV battery sensor.",
 "This sensor accesses the Khepera IV battery status. For a complete description\n"
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
 "  </controllers>\n\n",
 "Usable"
 );