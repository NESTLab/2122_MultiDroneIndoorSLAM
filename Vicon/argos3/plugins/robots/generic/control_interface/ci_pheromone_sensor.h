#ifndef CI_PHEROMONE_SENSOR_H
#define CI_PHEROMONE_SENSOR_H

#include <map>

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/simulator/media/pheromone_medium.h>


using namespace argos;

class CCI_PheromoneSensor : public CCI_Sensor {
public:

    typedef CPheromoneMedium::PheromoneMap TReadings;

    const CCI_PheromoneSensor::TReadings& GetReadings() const;

    const CVector3 GetGradient() const;

#ifdef ARGOS_WITH_LUA
    void CreateLuaState(lua_State* pt_lua_state) override;

    void ReadingsToLuaState(lua_State* pt_lua_state) override;
#endif

protected:

    CCI_PheromoneSensor::TReadings m_tReadings;
    CVector3 m_cGradient;

 };


#endif