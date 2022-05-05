#include "id_qtuser_functions.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() {
   RegisterUserFunction<CIDQTUserFunctions,CKheperaIVEntity>(&CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

std::string CIDQTUserFunctions::getLOSMessage(CKheperaIVRos& controller)
{
  std::vector<std::string> list_of_robots;
  controller.getListOfLOSRobots(list_of_robots);
  
  std::string robot_name_concat = "";
  for(int i = 0; i<list_of_robots.size(); i++)
    robot_name_concat += list_of_robots.at(i) + "   ";

  return robot_name_concat;
}

void CIDQTUserFunctions::Draw(CKheperaIVEntity& c_entity) {
   CKheperaIVRos& cController = dynamic_cast<CKheperaIVRos&>(c_entity.GetControllableEntity().GetController());

   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   DrawText(CVector3(0.10, 0.0, 0.3),   // position
            c_entity.GetId()); // text
   

   std::string connected_robots = "connected: " + getLOSMessage(cController);
   DrawText(CVector3(0.10, 0.05, 0.3),   // position
            connected_robots); // text

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "id_qtuser_functions")
