#include "pc_qt_user_functions.h"
#include <argos3/core/control_interface/ci_controller.h>

#include <argos3/plugins/controllers/khepera_diffusion/khepera_diffusion.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <algorithm>

/****************************************/
/****************************************/

CPCQTUserFunctions::CPCQTUserFunctions() :
   m_cPCLF(CMasterLoopFunctions::GetLoopFunction<CPCLoopFunctions>("pc_loop_functions"))
   {
}

/****************************************/
/****************************************/

void CPCQTUserFunctions::DrawInWorld() {
    
   std::map<std::string, CVector2> tTargetPos = m_cPCLF.GetTargetPos();
   std::map<std::string, CVector2> tDepotPos = m_cPCLF.GetDepotPos();
   std::map<std::string, UInt32> m_tTargetUpdated = m_cPCLF.GetTargetUpdated();
   UInt32 unTime = m_cPCLF.GetSimulator().GetSpace().GetSimulationClock();
   for(const auto& cPair: tTargetPos){
        CVector3 cPos;
        UInt8 unRed = std::min((unTime-m_tTargetUpdated[cPair.first])/4, UInt32(255));

        cPos.Set(cPair.second.GetX(), cPair.second.GetY(), 0.01);
        DrawCircle(cPos, CQuaternion(CRadians(0), CVector3::Z), 0.1, CColor(unRed, 0, 0));
   }

   for(const auto& cPair: tDepotPos){
        CVector3 cPos;
        cPos.Set(cPair.second.GetX(), cPair.second.GetY(), 0.01);
        DrawCircle(cPos, CQuaternion(CRadians(0), CVector3::Z), 0.1, CColor(0, 255, 0));
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CPCQTUserFunctions, "pc_qt_user_functions")
