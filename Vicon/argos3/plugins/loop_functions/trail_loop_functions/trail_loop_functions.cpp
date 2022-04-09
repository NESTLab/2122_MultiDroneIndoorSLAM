#include "trail_loop_functions.h"

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

void CTrailLoopFunctions::Init(TConfigurationNode& t_tree) {
   /*
    * Go through all the robots in the environment
    * and create an entry in the waypoint map for each of them
    */
  try{
   /* Get the map of all kheperas from the space */
    CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("KheperaIV");
    /* Go through them */
    for(const auto& it: tFBMap){
       /* Create a pointer to the current KheperaIV */
       CKheperaIVEntity* pcFB = any_cast<CKheperaIVEntity*>(it.second);
       /* Create a waypoint vector */
       m_tWaypoints[pcFB] = std::vector<CVector3>();
       /* Add the initial position of the KheperaIV */
       m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
    }
  }catch(CARGoSException& ex){
    return;
  }
}

/****************************************/
/****************************************/

void CTrailLoopFunctions::Reset() {
   /*
    * Clear all the waypoint vectors
    */
   try{
     /* Get the map of all kheperas from the space */
     CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("KheperaIV");
     /* Go through them */
       for(const auto& it: tFBMap){
           /* Create a pointer to the current kheperas */
           CKheperaIVEntity* pcFB = any_cast<CKheperaIVEntity*>(it.second);
           /* Clear the waypoint vector */
           m_tWaypoints[pcFB].clear();
           /* Add the initial position of the kheperas */
           m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
       }
   }catch(CARGoSException& ex){
       return;
   }
}

/****************************************/
/****************************************/

void CTrailLoopFunctions::PostStep() {
   /* Get the map of all kheperas from the space */
   try{
    CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("KheperaIV");
    /* Go through them */
     for(const auto& it: tFBMap) {
        /* Create a pointer to the current KheperaIV */
        CKheperaIVEntity* pcFB = any_cast<CKheperaIVEntity*>(it.second);
        /* Add the current position of the KheperaIV if it's sufficiently far from the last */
        if(m_tWaypoints[pcFB].empty()){
          m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
        }else if(SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
                          m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
           m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
           if(m_tWaypoints[pcFB].size() > 50){
            m_tWaypoints[pcFB].erase(m_tWaypoints[pcFB].begin());
           }
        }
     }
   }catch(CARGoSException& ex){
    return;
  }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTrailLoopFunctions, "trail_loop_functions")
