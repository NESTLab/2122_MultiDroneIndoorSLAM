#ifndef TRACKING_KHEPERA_MODEL_H
#define TRACKING_KHEPERA_MODEL_H

namespace argos {
    class CEmbodiedEntity;
}

#include <argos3/plugins/simulator/physics_engines/tracking/tracking_model.h>
#include <argos3/plugins/utility/networking/moveable_tcp_socket.h>

#include <argos3/core/utility/math/cylinder.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>

#include <mutex>
#include <thread>

typedef struct SThreadInfo{
    bool run;
    std::vector<CByteArray> cSendMessages;
    std::vector<CByteArray> cReadMessages;
    std::mutex cMessagesMutex;
    CMTCPSocket* cConnection;
} TThreadInfo;

class CTrackingKheperaModel : public CTrackingModel {
public:
    typedef std::map<UInt8, bool> TRealIRCliffDetectors;

public:

    CTrackingKheperaModel(CTrackingEngine& c_engine,
                         CKheperaIVEntity& c_f_entity);

    ~CTrackingKheperaModel() override;

    void CalculateBoundingBox() override;

    void UpdateFromEntityStatus() override;

    void UpdateEntityStatus() override;

    void static ThreadCallback(TThreadInfo* t_thread_info);

    bool CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray) override;

protected:
    CCylinder        m_cShape;
	CKheperaIVEntity&  m_cKheperaEntity;
	CWheeledEntity&  m_cWheeledEntity;
	const Real*      m_fCurrentWheelVelocity;

    TRealIRCliffDetectors   m_tRealCliffDetect;
    bool m_bFirstPass; 

	TThreadInfo m_tThreadInfo;
    std::thread m_cSendThread;
};

#endif
