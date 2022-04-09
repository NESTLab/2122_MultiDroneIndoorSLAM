#ifndef TRACKING_MODEL_H
#define TRACKING_MODEL_H

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/plugins/utility/networking/moveable_tcp_socket.h>
#include <argos3/plugins/utility/datatypes/reservable_buffer.h>

class CTrackingEngine;

using namespace argos;

class CTrackingModel : public CPhysicsModel {
public:

    CTrackingModel(CTrackingEngine& c_engine,
                  CEmbodiedEntity& c_entity);

    void UpdateFromEntityStatus() override;

    void UpdateEntityStatus() override;

    void MoveTo(const CVector3& c_position, const CQuaternion& c_orientation) override;

    bool IsCollidingWithSomething() const override;

    /**
    * Updates the origin anchor associated to the embodied entity.
    */
    virtual void UpdateOriginAnchor(SAnchor& s_anchor);

    virtual bool CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray);

    CVector3 GetPosition();

    CQuaternion GetOrientation();

    void SetPosition(CVector3 c_position);

    void SetOrientation(CQuaternion c_orientation);

    void SetConnection(CMTCPSocket c_connection);

    void SetTracked();

    void SetConnected();

    bool IsTracked();

    bool IsConnected();

    inline CByteArray* GetSensorData(const std::string& str_name){
        return m_tSensorData.Read(str_name);
    }

    inline void ReserveSensor(const std::string& str_name, UInt16 un_size, UInt8 un_fill = 0){
        m_tSensorData.ReserveSpace(str_name, un_size);
        m_tSensorData.Write(str_name, CByteArray(un_size, un_fill));
    }

protected:
    bool m_bConnected;
    bool m_bTracked;
    CVector3 m_cPosition;
    CQuaternion m_cOrientation;
    CMTCPSocket m_cConnection;   
    CReservableBuffer m_tSensorData;
};

#endif
