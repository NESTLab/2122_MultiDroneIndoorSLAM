#include "tracking_engine.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/plugins/dynamic_loading.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>

#include <argos3/plugins/simulator/physics_engines/tracking/tracking_khepera_model.h>
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_box_model.h>
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_cylinder_model.h>
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_kilobot_model.h>

/****************************************/
/****************************************/

std::vector<std::string> CTrackingEngine::ValidTypes = {"Khepera","Kilobot","Box","Cylinder"};

void CTrackingEngine::Init(TConfigurationNode& t_tree) {
    std::string strId = "tracking_engine", strUpdaterType;

    GetNodeAttributeOrDefault(t_tree, "id", strId, strId);
    SetId(strId);

    // set and init the updaters
    m_cControllerAssigner.Init(GetNode(t_tree, "assigners"));
    
    InitUpdaters(GetNode(t_tree, "updaters"));

    //RabOptions
    if(NodeExists(t_tree, "rab")){
        TConfigurationNode& tRABTree = GetNode(t_tree, "rab");
        TConfigurationNodeIterator itSubNode;
    
        for(itSubNode = itSubNode.begin(&tRABTree);
            itSubNode != itSubNode.end();
            ++itSubNode){

            if(IsValidType(itSubNode->Value())){
                UInt16 unDatasize;
                Real fRange;
                GetNodeAttribute(*itSubNode, "data_size", unDatasize);
                GetNodeAttribute(*itSubNode, "range", fRange);
                m_tRABOptions[itSubNode->Value()] = std::pair<UInt16, Real>(unDatasize, fRange);
            }
        }
    }
}

/****************************************/
/****************************************/

void CTrackingEngine::InitUpdaters(TConfigurationNode& t_tree){
    std::string strLibrary;
    CTrackingUpdater* cpUpdater;
    TConfigurationNodeIterator itSubNode;

    // iterate through the updaters
    for(itSubNode = itSubNode.begin(&t_tree);
        itSubNode != itSubNode.end();
        ++itSubNode) {
        GetNodeAttributeOrDefault(*itSubNode, "library", strLibrary, strLibrary);
        if(! strLibrary.empty()){
            CDynamicLoading::LoadLibrary(strLibrary);
        }

        cpUpdater = CFactory<CTrackingUpdater>::New(itSubNode->Value());
        cpUpdater->Init(*itSubNode);
        m_tUpdaters.push_back(cpUpdater);
    }
}

/****************************************/
/****************************************/

void CTrackingEngine::Reset() {
    for(auto& tpUpdater: m_tUpdaters){
        tpUpdater->Reset();
    }
}

/****************************************/
/****************************************/

void CTrackingEngine::Destroy() {
    for(auto tpUpdater: m_tUpdaters){
        tpUpdater->Destroy();
        delete tpUpdater;
    }

    m_cControllerAssigner.Destroy();
}

/****************************************/
/****************************************/

std::string CTrackingEngine::GetRobotTypeFromName(const std::string& str_sub_name){
    std::stringstream cStringStream;
    std::string strType;

    cStringStream.str(str_sub_name);
    std::getline(cStringStream, strType, '_');
    return strType;
}

/****************************************/
/****************************************/

bool CTrackingEngine::IsValidType(const std::string& str_type){
    return std::find(ValidTypes.begin(), ValidTypes.end(), str_type)!=ValidTypes.end();
}

/****************************************/
/****************************************/

void CTrackingEngine::Update() { 
    /* Update the physics state from the entities */
    for(const auto& it : m_tEntityMap){
        if(it.second){
            it.second->UpdateFromEntityStatus();
        }
    }

    // update each from each updaters
    for(const auto& tpUpdater: m_tUpdaters){
        TRobotInfoList tRobotInfoList = tpUpdater->Update();
        std::string strRobotOptions;
        // LOG << "Number of updates " << tRobotInfoList.size() << std::endl;

        for(const auto& tRobotInfo: tRobotInfoList){
            // skip if not a valid type
            if(!IsValidType(tRobotInfo.strType)){
                LOGERR << "Robot Type is '" << tRobotInfo.strType 
                       << "' and is an invalid type." << std::endl;
            }else{
                UpdateRobot(tRobotInfo);
            }
        }
    }

    for(const auto& it: m_tEntityMap) {
        if(it.second){
            it.second->UpdateEntityStatus();
        }
    }
}

/****************************************/
/****************************************/

void CTrackingEngine::CreateRobot(const std::string& str_type, 
                                  const std::string& str_name,
                                  const std::string& str_options){
    //TODO: Move these to model
    const std::string strController = GetController(str_type);
    LOG << str_name << " is being assigned the \"" 
        << strController << "\" controller " << std::endl;

    if(str_type == "Khepera"){
        Real fRange = 2.0;
        UInt16 unDatasize = 300;

        if(m_tRABOptions.find(str_type) != m_tRABOptions.end()){
            unDatasize = m_tRABOptions[str_type].first;
            fRange = m_tRABOptions[str_type].second;
        }
        CKheperaIVEntity *pcRobot = new CKheperaIVEntity(str_name, 
                                                         strController, 
                                                         CVector3(), 
                                                         CQuaternion(), 
                                                         fRange, 
                                                         unDatasize);
        CallEntityOperation<CSpaceOperationAddEntity, CSpace, void>
            (CSimulator::GetInstance().GetSpace(), *pcRobot);
    } else if(str_type == "Kilobot"){
        Real fRange = 0.1;

        if(m_tRABOptions.find(str_type) != m_tRABOptions.end()){
                        fRange = m_tRABOptions[str_type].second;
        }
        CKilobotEntity *pcKilobot = new CKilobotEntity(str_name, 
                                                       strController,
                                                       CVector3(), 
                                                       CQuaternion(),
                                                       fRange);
        CallEntityOperation<CSpaceOperationAddEntity, CSpace, void>
                (CSimulator::GetInstance().GetSpace(), *pcKilobot);
    } else if(str_type == "Box"){
        //TODO: Better error handling here
        //TODO: Add moveable and mass option
        CVector3 cSize(1,1,1);
        bool bError = false;
        if (str_options != ""){
            std::string strOption;
            std::stringstream cOptionsStream;
            cOptionsStream.str(str_options);

            std::getline(cOptionsStream, strOption, '_');
            cSize.SetX(FromString<Real>(strOption));

            std::getline(cOptionsStream, strOption, '_');
            cSize.SetY(FromString<Real>(strOption));

            std::getline(cOptionsStream, strOption, '_');
            cSize.SetZ(FromString<Real>(strOption));
        } else {
            bError = true;
        }

        if(bError){
            LOGERR << "Error with box '" << str_name 
                   << "' with options '" << str_options 
                   << "' Options should be in format 'XSize,YSize,ZSize'" 
                   << "(Length, Width, Height). Got \"" 
                   << str_options << "\"" << std::endl;
        }else{
            LOG << "Creating box \"" << str_name <<"\" with size " << cSize << std::endl; 
        }
        CBoxEntity *pcBox = new CBoxEntity(str_name,
                                           CVector3(), 
                                           CQuaternion(),
                                           true,
                                           cSize);
        CallEntityOperation<CSpaceOperationAddEntity, CSpace, void>
                (CSimulator::GetInstance().GetSpace(), *pcBox);
    } else if(str_type == "Cylinder"){
        Real fHeight;
        Real fRadius;
        bool bError = false;
        if (str_options == ""){
            std::string strOption;
            std::stringstream cOptionsStream;
            cOptionsStream.str(str_options);

            std::getline(cOptionsStream, strOption, ',');
            fRadius = FromString<Real>(strOption);

            std::getline(cOptionsStream, strOption, ',');
            fHeight = FromString<Real>(strOption);
        } else {
            bError = true;
        }

        if(bError){
            LOGERR << "Error with cylinder '" << str_name 
                   << "' with options '" << str_options 
                   << "' Options should be in format 'Radius, Height'" 
                   << std::endl;
        }

        CCylinderEntity *pcCylinder = new CCylinderEntity(str_name,
                                           CVector3(), 
                                           CQuaternion(),
                                           true,
                                           fRadius,
                                           fHeight);
        CallEntityOperation<CSpaceOperationAddEntity, CSpace, void>
                (CSimulator::GetInstance().GetSpace(), *pcCylinder);
    }
}

/****************************************/
/****************************************/

void CTrackingEngine::UpdateRobot(const TRobotInfo& t_robot_info){
    // if the robot does not currently exist, create it
    if(!HasRobot(t_robot_info.strName))
    {
        CreateRobot(t_robot_info.strType, t_robot_info.strName, t_robot_info.strOptions);
   }
    m_tEntityMap[t_robot_info.strName]->SetPosition(t_robot_info.cTrans);
    m_tEntityMap[t_robot_info.strName]->SetOrientation(t_robot_info.cQuat);
    m_tEntityMap[t_robot_info.strName]->SetTracked();
}

/****************************************/
/****************************************/

bool CTrackingEngine::HasRobot(const std::string& str_name){
    auto cEntityMapEntry = m_tEntityMap.find(str_name);
    return cEntityMapEntry != m_tEntityMap.end();
}

/****************************************/
/****************************************/

void CTrackingEngine::PostSpaceInit() {
}

/****************************************/
/****************************************/

size_t CTrackingEngine::GetNumPhysicsModels() {
    return m_tEntityMap.size();
}

/****************************************/
/****************************************/

bool CTrackingEngine::AddEntity(CEntity& c_entity) {
    std::string strName = c_entity.GetId();
    std::string strType = c_entity.GetTypeDescription();
    auto cEntityMapEntry = m_tEntityMap.find(strName);

    // LOG << "attempting to add robot of type " << strType << std::endl;

    if(cEntityMapEntry != m_tEntityMap.end())
    {
        LOGERR << "Model with entity name already exists";
        return false;
    }

    CKheperaIVEntity* pcKhepera = dynamic_cast<CKheperaIVEntity*>(&c_entity);
    if(pcKhepera != nullptr) {
        auto *cKheperaModel = new CTrackingKheperaModel(*this, *pcKhepera);
        m_tEntityMap[strName] = cKheperaModel;
        return true;
    }

    CKilobotEntity* pcKilobot = dynamic_cast<CKilobotEntity*>(&c_entity);
    if(pcKilobot != nullptr) {
        auto *cKheperaModel =new CTrackingKilobotModel(*this, *pcKilobot);
        m_tEntityMap[strName] = cKheperaModel;
        return true;
    }

    auto* pcWall = dynamic_cast<CBoxEntity*>(&c_entity);
    if(pcWall != nullptr) {
        auto *cWallModel = new CTrackingBoxModel(*this, *pcWall);
        m_tEntityMap[strName] = cWallModel;
        return true;
    }

    auto* pcCyl = dynamic_cast<CCylinderEntity*>(&c_entity);
    if(pcCyl != nullptr) {
        auto *cCylModel = new CTrackingCylinderModel(*this, *pcCyl);
        m_tEntityMap[strName] = cCylModel;
        return true;
    }

    return false;
}

/****************************************/
/****************************************/

bool CTrackingEngine::RemoveEntity(CEntity& c_entity) {
    const std::string& strName = c_entity.GetId();
    auto cEntityMapEntry = m_tEntityMap.find(strName);

    if(cEntityMapEntry == m_tEntityMap.end())
    {
        LOGERR << "Model with entity name does note exist";
        return false;
    }
    cEntityMapEntry->second->GetEmbodiedEntity().RemovePhysicsModel(GetId());
    m_tEntityMap.erase(cEntityMapEntry);
    return true;
}

/****************************************/
/****************************************/

void CTrackingEngine::CheckIntersectionWithRay(TEmbodiedEntityIntersectionData& t_data,
                                               const CRay3& c_ray) const {
    Real fTOnRay;
    // check intersection for each model in the engine
    for(auto it = m_tEntityMap.begin();
        it != m_tEntityMap.end(); ++it) {
        if(it->second && 
           it->second->CheckIntersectionWithRay(fTOnRay, c_ray) &&
           fTOnRay < 1){
            t_data.emplace_back(&it->second->GetEmbodiedEntity(),fTOnRay);
         }
    } 
}

/****************************************/
/****************************************/

REGISTER_PHYSICS_ENGINE(CTrackingEngine, 
                        "tracking", 
                        "Chris Cormier", 
                        "0.1",
                        "A Vicon tracking engine",
                        "Communicates with the Vicon DataStrem API to \
                        find the locations of the various\n"
                        "objects in the arena.",
                        "Debug");
