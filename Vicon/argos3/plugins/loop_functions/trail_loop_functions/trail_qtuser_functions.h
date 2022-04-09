#ifndef TRAJECTORY_QTUSER_FUNCTIONS_H
#define TRAJECTORY_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

using namespace argos;

class CTrailLoopFunctions;

class CTrailQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CTrailQTUserFunctions();

   void DrawInWorld() override;

private:

   void DrawWaypoints(const std::vector<CVector3>& c_waypoints);

private:

   CTrailLoopFunctions& m_cTrajLF;

};

#endif
