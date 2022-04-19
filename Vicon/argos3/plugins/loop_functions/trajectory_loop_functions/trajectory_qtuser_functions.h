#ifndef TRAJECTORY_QTUSER_FUNCTIONS_H
#define TRAJECTORY_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

using namespace argos;

class CTrajectoryLoopFunctions;

class CTrajectoryQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CTrajectoryQTUserFunctions();

   void DrawInWorld() override;

private:

   void DrawWaypoints(const std::vector<CVector3>& c_waypoints);

private:

   CTrajectoryLoopFunctions& m_cTrajLF;

};

#endif
