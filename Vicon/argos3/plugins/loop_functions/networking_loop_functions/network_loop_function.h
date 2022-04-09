#ifndef NETWORK_LOOP_FUNCTION_H
#define NETWORK_LOOP_FUNCTION_H

#include <argos3/core/simulator/loop_functions.h>
#include <mutex>
#include <thread>
#include <argos3/plugins/utility/networking/moveable_tcp_socket.h>

using namespace argos;

class CNetworkLoopFunction : public CLoopFunctions{

public:

    void Init(TConfigurationNode& t_tree) override;

    void Reset() override;

    void Destroy() override;

    void PreStep() override;

    void PostStep() override;

    bool IsExperimentFinished() override;

    void PostExperiment() override;
    
    void static ThreadCallback(CNetworkLoopFunction* c_network_loop_function);

    bool m_bRun;
    UInt32 m_unPort = 22222;
    
    std::vector<CMTCPSocket> m_cSocketVector;
    std::vector<std::string> m_cNewNames;
    std::mutex m_cBufferMutex;
    std::thread m_cListenThread;

private:

};


#endif
