#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>

using namespace argos;

class CTrajectoryLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CKheperaIVEntity*, std::vector<CVector3> > TWaypointMap;
   TWaypointMap m_tWaypoints;
   
public:
   void Init(TConfigurationNode& t_tree) override;

   void Reset() override;

   void PostStep() override;

   inline const TWaypointMap& GetWaypoints() const {
      return m_tWaypoints;
   }

private:

};

#endif
