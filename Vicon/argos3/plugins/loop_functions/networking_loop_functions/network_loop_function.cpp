#include "network_loop_function.h"

#include <argos3/plugins/simulator/physics_engines/tracking/tracking_engine.h>
// #include <argos3/plugins/simulator/physics_engines/tracking/tracking_khepera_model.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>

/****************************************/
/****************************************/

// callback function so listening for new robots can occur in separate thread 
void CNetworkLoopFunction::ThreadCallback(CNetworkLoopFunction* c_network_loop_function){
    //needed to use LOG and LOGERR in a new thread
    LOG.AddThreadSafeBuffer();
    LOGERR.AddThreadSafeBuffer();

    // .1 sec sleep
    struct timespec sSleep;
    sSleep.tv_sec = 0;
    sSleep.tv_nsec = 10000000;

    // create a new buffer
    CByteArray buffer(0);
    CMTCPSocket listener;
    listener.Listen(22222);

    while(c_network_loop_function->m_bRun){
        CMTCPSocket newConnection;
        while(listener.MessageAvalible()){

            listener.Accept(newConnection);

            LOG <<"new connection from " 
                << newConnection.GetAddress() << std::endl;

            //Request name
            buffer.Clear();
            UInt8 zero = 0;
            buffer << zero;

            // send request
            newConnection.SendByteArray(buffer);
            buffer.Clear();
            //receive repsonse
            newConnection.ReceiveByteArray(buffer);

            std::string name;
            buffer << '\0';
            buffer >> name;
            LOG << "Connection name: " << name << std::endl;

            // get and check robot type
            std::string strType = CTrackingEngine::GetRobotTypeFromName(name);
            bool bValid = CTrackingEngine::IsValidType(strType);

            if(!bValid){
                LOG << "Robot Type is '" << strType << "' and is an invalid type." << std::endl;
                newConnection.Disconnect();
                continue;
            }

            // lock the mutex
            std::lock_guard<std::mutex> lock(c_network_loop_function->m_cBufferMutex);
            // push new information on to the stack
            c_network_loop_function->m_cNewNames.push_back(name);
            c_network_loop_function->m_cSocketVector.push_back(std::move(newConnection));
        }
        nanosleep(&sSleep, nullptr);
    }
}

/****************************************/
/****************************************/

void CNetworkLoopFunction::Init(TConfigurationNode& t_tree) {
    GetNodeAttributeOrDefault(t_tree, "port", m_unPort, m_unPort);
    // start callback thread
    m_bRun = true;
    m_cListenThread = std::thread(ThreadCallback, this);

    LOG << "Finished Init basic networking" << std::endl;
}

/****************************************/
/****************************************/

void CNetworkLoopFunction::Reset() {

}

/****************************************/
/****************************************/

void CNetworkLoopFunction::Destroy() {
    // close thread
    m_bRun = false;
    m_cListenThread.join();

}

/****************************************/
/****************************************/

void CNetworkLoopFunction::PreStep() {
    // lock the mutex
    std::unique_lock<std::mutex> lock(m_cBufferMutex);

    // move protected structures into a working structure
    std::vector<std::string> workingConnectionNames = m_cNewNames;
    std::vector<CMTCPSocket> workingSocketVector = std::move(m_cSocketVector);
    m_cSocketVector.clear();
    m_cNewNames.clear();

    lock.unlock();

    // if there are new connections to parse
    while(!workingConnectionNames.empty()){
        // pop the current data off the stack
        CMTCPSocket connection = std::move(workingSocketVector.back());
        std::string strName = workingConnectionNames.back();
        workingSocketVector.pop_back();
        workingConnectionNames.pop_back();

        // get the physics engine 
        CTrackingEngine* cTrackingEngine = dynamic_cast<CTrackingEngine*>(&(CSimulator::GetInstance().GetPhysicsEngine("tracking_engine")));
        
        // create robot if it does not currently exist
        if(!cTrackingEngine->HasRobot(strName)){
            std::string strType = CTrackingEngine::GetRobotTypeFromName(strName);
            cTrackingEngine->CreateRobot(strType, strName);
        }

        // get the tracking model
        CKheperaIVEntity& pcRobot = (dynamic_cast<CKheperaIVEntity&>(CSimulator::GetInstance().GetSpace().GetEntity(strName)));
        CTrackingModel& model = dynamic_cast<CTrackingModel&>(pcRobot.GetEmbodiedEntity().GetPhysicsModel("tracking_engine"));

        if(model.IsConnected()){
            // if the model already has a connection, throw out the new one
            LOGERR << "Model with name '" << strName << "' already has a connection. "
                   << "Closing new connection from " << connection.GetAddress() << std::endl;
            connection.Disconnect();
        } else {
            // otherwise move the connection to the 
            LOG << "Adding connection on socket" << connection.GetAddress() << " to robot '" 
                   << strName << "'" << std::endl;
            CByteArray message;
            model.SetConnection(std::move(connection));
            model.SetConnected();
        }
    }
}

/****************************************/
/****************************************/

void CNetworkLoopFunction::PostStep(){

}
/****************************************/
/****************************************/

bool CNetworkLoopFunction::IsExperimentFinished() {
    return false;
}

/****************************************/
/****************************************/

void CNetworkLoopFunction::PostExperiment() {

}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CNetworkLoopFunction, "BaseNetworking");
