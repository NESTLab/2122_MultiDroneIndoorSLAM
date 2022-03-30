#pragma once

#include <robot_state_machine.h>
#include <unordered_map>

class TeamScheduler{
public:
   TeamScheduler(ros::NodeHandle &nh, const std::string& robot_name, ROLE role, const std::string& parent_name, const std::string& child_name, bool testing);
   ~TeamScheduler();
   
   void exec();
   void step();

   RobotState* getStatePtr(uint64_t un_id);
   void setTeamMacroState(TEAM_STATES state){ 
         robot_state = state;}

private:
   ros::NodeHandle nh_;
   bool testing_mode_;
   std::string robot_name_;

   void addStates();
   void addState(RobotState* RobotStatePtr);
   TEAM_STATES robot_state;
   bool new_state_request;

   void setInitialState(uint64_t un_state);
   void setErrorState();

   RobotState* current_state_ptr;
   std::unordered_map<uint64_t, RobotState*> MACRO_STATE_PTR_MAP;
};
