#include "pc_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/core/utility/string_utilities.h>
#include <cmath>

/****************************************/
/****************************************/

void CPCLoopFunctions::Init(TConfigurationNode& t_tree) {
    TConfigurationNodeIterator cCurrNode;
    std::string strId;
    Real punCoordinates[2];
    CVector2 cPos;
    try {
        GetNodeAttribute(t_tree, "file", m_strOutputFile);
        for(cCurrNode = cCurrNode.begin(&t_tree);
            cCurrNode != cCurrNode.end();
            ++cCurrNode){
            if(cCurrNode->Value() == "target"){
                GetNodeAttribute(*cCurrNode, "pos", strId);
                ParseValues<Real>(strId, 2, punCoordinates, ',');
                cPos.Set(punCoordinates[0], punCoordinates[1]);

                GetNodeAttribute(*cCurrNode, "id", strId);
                strId = "target_" + strId;
                m_tTargetPos[strId] = cPos;
            }else if(cCurrNode->Value() == "depot"){
                GetNodeAttribute(*cCurrNode, "pos", strId);
                ParseValues<Real>(strId, 2, punCoordinates, ',');
                cPos.Set(punCoordinates[0], punCoordinates[1]);

                GetNodeAttribute(*cCurrNode, "id", strId);
                strId = "depot_" + strId;
                m_tDepotPos[strId] = cPos;
            }
        }

        Real fMinDist, fDist;
        std::string strMinDepot;
        // calculate closest depot for all targets
        for(auto cTPair: m_tTargetPos){
            fMinDist = std::numeric_limits<Real>::infinity();
            strMinDepot = "";

            // get closest depot
            for(auto cDPair: m_tDepotPos){
                // get dist between depot and target
                fDist = (cTPair.second-cDPair.second).Length();
                // if less than prev min dist, set as min dist and min depot
                if(fDist < fMinDist){
                    fMinDist = fDist;
                    strMinDepot = cDPair.first;
                }
            }
            if(strMinDepot != ""){
                m_tTargetDepot[cTPair.first] = strMinDepot;
                m_tTargetUpdated[cTPair.first] = 0;
            }else{
                LOG << cTPair.first << " does not have nearest depot" << std::endl;
            }
        }
    }catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }

    // get diffusion params
    // get tolerance

    /* Open the file, erasing its contents */
    m_cOutput.open(m_strOutputFile, std::ios_base::trunc | std::ios_base::out);

}
/****************************************/
/****************************************/

void CPCLoopFunctions::InitVMs() {
    CSpace::TMapPerType* m_ptKheMap;
    try{
        m_ptKheMap = &(CSimulator::GetInstance().GetSpace().GetEntitiesByType("kheperaiv"));
    }catch(CARGoSException& ex){
        LOGERR << "Issue getting entities by type 'Khepera'" <<std::endl;
        return;
    }
    // get all the buzz vms
    for(const auto& it: *m_ptKheMap) {
        /* Create a pointer to the current khepera and its controller*/
        auto* pcKE = any_cast<CKheperaIVEntity*>(it.second);
        CBuzzController* pcController = dynamic_cast<CBuzzController*>(
                                      &(pcKE->GetControllableEntity().GetController()));

        if(std::find(m_tControllers.begin(), m_tControllers.end(), pcController) != m_tControllers.end()){
            continue;
        }
        m_tControllers.push_back(pcController);

        buzzvm_t tBuzzVM = pcController->GetBuzzVM();
        // create target table
        buzzvm_pushs(tBuzzVM, buzzvm_string_register(tBuzzVM, "targets", 1));
        buzzvm_pusht(tBuzzVM);
        buzzobj_t tTargetTable = buzzvm_stack_at(tBuzzVM, 1);
        buzzvm_gstore(tBuzzVM);
        /* Fill the target table */
        buzzobj_t tTarget;
        CVector3 cPos3;
        for(auto cPair: m_tTargetPos) {
             /* Create table for i-th read */
             buzzvm_pusht(tBuzzVM);
             tTarget = buzzvm_stack_at(tBuzzVM, 1);
             buzzvm_pop(tBuzzVM);
             /* Fill in the pos */
             TablePut(tBuzzVM, tTarget, "pos", cPair.second);
             TablePut(tBuzzVM, tTarget, "nearest_depot", m_tTargetDepot[cPair.first]);
             /* Store read table in the target table */
             TablePut(tBuzzVM, tTargetTable, cPair.first, tTarget);
        }

        // create depot table
        buzzvm_pushs(tBuzzVM, buzzvm_string_register(tBuzzVM, "depots", 1));
        buzzvm_pusht(tBuzzVM);
        buzzobj_t tDepotTable = buzzvm_stack_at(tBuzzVM, 1);
        buzzvm_gstore(tBuzzVM);
        /* Fill the depot table */
        buzzobj_t tDepot;
        for(auto cPair: m_tDepotPos) {
             /* Create table for current tuple */
             buzzvm_pusht(tBuzzVM);
             tDepot = buzzvm_stack_at(tBuzzVM, 1);
             buzzvm_pop(tBuzzVM);
             /* Fill in the pos */
             TablePut(tBuzzVM, tDepot, "pos", cPair.second);
             /* Store read table in the depot table */
             TablePut(tBuzzVM, tDepotTable, cPair.first, tDepot);
        }

    }
    // push diffusion
    // push tolerance
    // 
    // remember to remove depot calc from bzz after implementing this push
}

/****************************************/
/****************************************/

void CPCLoopFunctions::Reset() {
    /* Close the file */
    m_cOutput.close();
    /* Open the file, erasing its contents */
    m_cOutput.open(m_strOutputFile, std::ios_base::trunc | std::ios_base::out);
    m_cOutput << "# clock\t" << std::endl;

}

/****************************************/
/****************************************/

void CPCLoopFunctions::Destroy() {
    /* Close the file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

void CPCLoopFunctions::PreStep() {
    InitVMs();
}

/****************************************/
/****************************************/

void CPCLoopFunctions::PostStep() {
    CSpace::TMapPerType* m_ptKheMap;
    try{
        m_ptKheMap = &(CSimulator::GetInstance().GetSpace().GetEntitiesByType("kheperaiv"));
    }catch(CARGoSException& ex){
        LOGERR << "Issue getting entities by type 'Khepera'" <<std::endl;
        return;
    }

    std::map<std::string,CBuzzController> tControllers;
    // get all the buzz vms
    for(const auto& it: *m_ptKheMap) {
        auto* pcKE = any_cast<CKheperaIVEntity*>(it.second);
        CCI_Controller* pcController = &(pcKE->GetControllableEntity().GetController());
        auto cController  = dynamic_cast<CBuzzController*>(pcController);
        
        CVector2 cGoal, cPosition;
        std::string strGoal;
        if(!GetGoal(cController->GetBuzzVM(), strGoal, cGoal)){
            LOGERR << "Issue getting goal position for " << pcKE->GetId() << std::endl;
            LOGERR.Flush();
            continue;
        }

        pcKE->GetEmbodiedEntity().GetOriginAnchor().Position.ProjectOntoXY(cPosition);
        Real fDist = (cGoal - cPosition).Length();
        Real fTolerance;
        if(!GetTolerance(cController->GetBuzzVM(), fTolerance)){
            LOGERR << "Issue getting TOLERANCE for " << pcKE->GetId() << std::endl;
            LOGERR.Flush();
            continue;
        }

        // better detection
        if(fDist < fTolerance + 0.005){
            m_tTargetUpdated[strGoal] = GetSimulator().GetSpace().GetSimulationClock();
            m_cOutput << strGoal << "," << pcKE->GetId() << "," 
                      << GetSimulator().GetSpace().GetSimulationClock()
                      << std::endl;
            LOGERR << strGoal << "," << pcKE->GetId() << "," 
                   << GetSimulator().GetSpace().GetSimulationClock()
                   << std::endl;
        }
    }


}

/****************************************/
/****************************************/

bool CPCLoopFunctions::GetGoal(buzzvm_t t_buzz_vm, std::string& strGoal, CVector2& cGoal){
    // Get the dest target name
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "pc", 1));
    buzzvm_gload(t_buzz_vm);
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "dest", 1));
    buzzvm_tget(t_buzz_vm);
    buzzobj_t tVar = buzzvm_stack_at(t_buzz_vm, 1);

    buzzvm_pop(t_buzz_vm);
    // check is the valid
    if (tVar->o.type == BUZZTYPE_STRING) {
        strGoal = tVar->s.value.str;
    }else{
        return false;
    }

    std::string strType = strGoal.substr(0,1) == "t" ? "targets" : "depots";
    // get target x position
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "pc", 1));
    buzzvm_gload(t_buzz_vm);
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, strType.c_str(), 1));
    buzzvm_tget(t_buzz_vm);
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, strGoal.c_str(), 1));
    buzzvm_tget(t_buzz_vm);
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "pos", 1));
    buzzvm_tget(t_buzz_vm);
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "x", 1));
    buzzvm_tget(t_buzz_vm);

    tVar = buzzvm_stack_at(t_buzz_vm, 1);
    buzzvm_pop(t_buzz_vm);
    if (tVar->o.type == BUZZTYPE_FLOAT) {
        cGoal.SetX(tVar->f.value);
    }else if(tVar->o.type == BUZZTYPE_INT){
        cGoal.SetX(tVar->i.value);
    }else{
        return false;
    }

    // get target x position
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "pc", 1));
    buzzvm_gload(t_buzz_vm);
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, strType.c_str(), 1));
    buzzvm_tget(t_buzz_vm);
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, strGoal.c_str(), 1));
    buzzvm_tget(t_buzz_vm);
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "pos", 1));
    buzzvm_tget(t_buzz_vm);
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "y", 1));
    buzzvm_tget(t_buzz_vm);

    tVar = buzzvm_stack_at(t_buzz_vm, 1);
    buzzvm_pop(t_buzz_vm);
    if (tVar->o.type == BUZZTYPE_FLOAT) {
        cGoal.SetY(tVar->f.value);
        return true;
    }else if(tVar->o.type == BUZZTYPE_INT){
        cGoal.SetY(tVar->i.value);
        return true;
    }else{
        return false;
    }
}

/****************************************/
/****************************************/

bool CPCLoopFunctions::GetTolerance(buzzvm_t t_buzz_vm, Real& fTolerance){
    // Get the dest target name
    buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "TOLERANCE", 1));
    buzzvm_gload(t_buzz_vm);

    buzzobj_t tVar = buzzvm_stack_at(t_buzz_vm, 1);
    buzzvm_pop(t_buzz_vm);
    if (tVar->o.type == BUZZTYPE_FLOAT) {
        fTolerance = tVar->f.value;
        return true;
    }else if(tVar->o.type == BUZZTYPE_INT){
        fTolerance = tVar->i.value;
        return true;
    }else{
        return false;
    }
}

/****************************************/
/****************************************/

buzzvm_state CPCLoopFunctions::TablePut(buzzvm_t t_buzzVM,
                                        buzzobj_t t_table,
                                        const std::string& str_key,
                                        buzzobj_t t_obj) {
   buzzvm_push(t_buzzVM, t_table);
   buzzvm_pushs(t_buzzVM, buzzvm_string_register(t_buzzVM, str_key.c_str(), 1));
   buzzvm_push(t_buzzVM, t_obj);
   buzzvm_tput(t_buzzVM);
   return t_buzzVM->state;
}

/****************************************/
/****************************************/

buzzvm_state CPCLoopFunctions::TablePut(buzzvm_t t_buzzVM,
                                        buzzobj_t t_table,
                                        const std::string& str_key,
                                        Real f_value) {
   buzzvm_push(t_buzzVM, t_table);
   buzzvm_pushs(t_buzzVM, buzzvm_string_register(t_buzzVM, str_key.c_str(), 1));
   buzzvm_pushf(t_buzzVM, f_value);
   buzzvm_tput(t_buzzVM);
   return t_buzzVM->state;
}

/****************************************/
/****************************************/

buzzvm_state CPCLoopFunctions::TablePut(buzzvm_t t_buzzVM,
                                        buzzobj_t t_table,
                                        const std::string& str_key,
                                        std::string str_value) {
   buzzvm_push(t_buzzVM, t_table);
   buzzvm_pushs(t_buzzVM, buzzvm_string_register(t_buzzVM, str_key.c_str(), 1));
   buzzvm_pushs(t_buzzVM, buzzvm_string_register(t_buzzVM, str_value.c_str(), 1));
   buzzvm_tput(t_buzzVM);
   return t_buzzVM->state;
}

/****************************************/
/****************************************/

buzzvm_state CPCLoopFunctions::TablePut(buzzvm_t t_buzzVM,
                                        buzzobj_t t_table,
                                        const std::string& str_key,
                                        const CVector2& c_vec) {
   buzzvm_push(t_buzzVM, t_table);
   buzzvm_pushs(t_buzzVM, buzzvm_string_register(t_buzzVM, str_key.c_str(), 1));
   buzzvm_pusht(t_buzzVM);
   buzzobj_t tVecTable = buzzvm_stack_at(t_buzzVM, 1);
   buzzvm_tput(t_buzzVM);
   TablePut(t_buzzVM, tVecTable, "x", c_vec.GetX());
   TablePut(t_buzzVM, tVecTable, "y", c_vec.GetY());
   return t_buzzVM->state;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CPCLoopFunctions, "pc_loop_functions")
