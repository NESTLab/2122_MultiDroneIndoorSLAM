#include "trail_qtuser_functions.h"
#include <argos3/plugins/loop_functions/master_loop_functions/master_loop_functions.h>
#include "trail_loop_functions.h"

/****************************************/
/****************************************/

CTrailQTUserFunctions::CTrailQTUserFunctions() :
   m_cTrajLF(CMasterLoopFunctions::GetLoopFunction<CTrailLoopFunctions>("trail_loop_functions"))
   {}

/****************************************/
/****************************************/

void CTrailQTUserFunctions::DrawInWorld() {
   /* Go through all the robot waypoints and draw them */
   for(const auto& it: m_cTrajLF.GetWaypoints()){
      DrawWaypoints(it.second);
   }
}

/****************************************/
/****************************************/

void CTrailQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd],
                       c_waypoints[unStart]));
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTrailQTUserFunctions, "trail_qtuser_functions")
