#ifndef ID_QTUSER_FUNCTIONS_H
#define ID_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <ros/ros.h>
#include "argos_bridge/losList.h"
#include <argos_bridge/src/plugins/controllers/kheperaiv_ros/kheperaiv_ros.h>


using namespace argos;

class CIDQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CIDQTUserFunctions();
   virtual ~CIDQTUserFunctions() {};
   
   void Draw(CKheperaIVEntity& c_entity);

private:
  std::string getLOSMessage(CKheperaIVRos& controller);

  ros::NodeHandle nodeHandle_;
  bool once_=true;
  int robot_name_length_ = 5;
   
};

#endif
