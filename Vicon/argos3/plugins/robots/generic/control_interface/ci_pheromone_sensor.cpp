#include "ci_pheromone_sensor.h"

const CCI_PheromoneSensor::TReadings& CCI_PheromoneSensor::GetReadings() const{
    return m_tReadings;
}

const CVector3 CCI_PheromoneSensor::GetGradient() const{
    return m_cGradient;
}

/****************************************/
/****************************************/

#ifdef ARGOS_WITH_LUA
    void CCI_PheromoneSensor::CreateLuaState(lua_State* pt_lua_state){

    }

    /****************************************/
    /****************************************/

    void CCI_PheromoneSensor::ReadingsToLuaState(lua_State* pt_lua_state){

    }

    /****************************************/
    /****************************************/
#endif


