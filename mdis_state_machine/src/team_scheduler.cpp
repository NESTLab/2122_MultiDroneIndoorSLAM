#include <team_scheduler.h>

TeamScheduler::TeamScheduler(ros::NodeHandle &nh, const std::string& robot_name, ROLE role, const std::string& parent_name, const std::string& child_name, bool testing):nh_(nh), testing_mode_(testing),robot_name_(robot_name)
{
    addStates();
    if (role == EXPLORER)
      setInitialState(GO_TO_EXPLORE);
    else if (role == RELAY)
      setInitialState(GO_TO_MEET);
    else
      setInitialState(DATA_CENTER_READY_TO_MEET);

    current_state_ptr->setParent(parent_name);
    current_state_ptr->setChild(child_name);
    current_state_ptr->setRobotRole(role);
}

TeamScheduler::~TeamScheduler()
{
   std::for_each(
      MACRO_STATE_PTR_MAP.begin(),
      MACRO_STATE_PTR_MAP.end(),
      [](std::pair<uint64_t, RobotState*> c_item){
         delete c_item.second;
      });
}

void TeamScheduler::addStates()
{
    addState(new Idle(nh_, testing_mode_));
    addState(new GoToExplore(nh_, testing_mode_));
    addState(new Explore(nh_, testing_mode_));
    addState(new GoToMeet(nh_, testing_mode_));
    addState(new TransitToMeet(nh_, testing_mode_));
    addState(new MergeMap(nh_, testing_mode_));
    addState(new DecideNextMeeting(nh_, testing_mode_));
    addState(new ReceiveNextMeeting(nh_, testing_mode_));
    addState(new EndMeeting(nh_, testing_mode_));
    addState(new GoToDumpData(nh_, testing_mode_));
    addState(new DataCenterReadyToMeet(nh_, testing_mode_));
    addState(new ErrorState(nh_, testing_mode_));
}

void TeamScheduler::addState(RobotState* pc_state) {
   if(MACRO_STATE_PTR_MAP.find(pc_state->getId()) == MACRO_STATE_PTR_MAP.end()) {
      MACRO_STATE_PTR_MAP[pc_state->getId()] = pc_state;
   }
   else {
      ROS_WARN_STREAM("["<<robot_name_<<" | mdis_state_machine | team_scheduler.cpp]: Duplicated state id :" << pc_state->getId());
   }
}

RobotState* TeamScheduler::getStatePtr(uint64_t un_id)
{
   auto pcState = MACRO_STATE_PTR_MAP.find(un_id);
   if(pcState != MACRO_STATE_PTR_MAP.end()) {
      return (pcState->second);
   }
   else {
      ROS_ERROR_STREAM("["<<robot_name_<<" | mdis_state_machine | team_scheduler.cpp]:Can't get state id " << un_id);
      auto pcState = MACRO_STATE_PTR_MAP.find(ERROR_STATE);
      if(pcState != MACRO_STATE_PTR_MAP.end()) {
          ROS_ERROR_STREAM("["<<robot_name_<<" | mdis_state_machine | team_scheduler.cpp]:Changing to ERROR_STATE " << un_id);
          return pcState->second;
      }
      else
      {
        ROS_ERROR_STREAM("["<<robot_name_<<" | mdis_state_machine | team_scheduler.cpp]: Error state not found. Creating and returning new error state.");
        return new ErrorState(nh_, false);
      }
   }
}

void TeamScheduler::setInitialState(uint64_t un_state) {
   current_state_ptr = getStatePtr(un_state);
   current_state_ptr->entryPoint();
   setTeamMacroState((TEAM_STATES) un_state);
}

void TeamScheduler::setErrorState()
{
  setTeamMacroState(ERROR_STATE);
  current_state_ptr = getStatePtr(ERROR_STATE);
  current_state_ptr->entryPoint();
  return;
}

void TeamScheduler::step() {
   /* Only execute if 'current' was initialized */
   if(current_state_ptr) {
      /* Attempt a transition, every state of every rover has its own transition() */
      TEAM_STATES newStateEnum = current_state_ptr->transition();
      RobotState* cNewState = getStatePtr(newStateEnum);

      if(cNewState != current_state_ptr) {
         /* Perform transition */
         current_state_ptr->exitPoint();
         if(!cNewState->entryPoint())
         {
            ROS_ERROR_STREAM("["<<robot_name_<<" | mdis_state_machine | team_scheduler.cpp]: Entry point to the state "<<newStateEnum<<" failed. Transitioning to ERROR_STATE");
            setErrorState();
            return;
         }

         setTeamMacroState(newStateEnum);
         current_state_ptr = cNewState;
      }
      /* Execute current state */
      current_state_ptr->step();
   }
   else {
      ROS_ERROR_STREAM("["<<robot_name_<<" | mdis_state_machine | team_scheduler.cpp]: The robot has not been initialized, you must call SetInitialState()");
   }
}

// /****************************************/
// /****************************************/

//UNDERSTANDING: Each robot has its own done() and this is what is checked to perform step()
void TeamScheduler::exec() {
   ros::Rate r(5); // 50Hz loop rate
   while(ros::ok())
   {
      step();
      ros::spinOnce();
      r.sleep();
   }
}


int main(int argc, char** argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle nh;

   bool testing = false;
   ROLE role;
   std::string parent_name, child_name;

   if (std::stoi(argv[1]) < 5)
   {
      ROS_WARN("Argument checking is turned off. Please verify if the arguments are: Role, parent_robot_name and child_robot_name (if applicable)");
      role = (ROLE)(std::stoi(argv[1]));
   }
   else
   {
     ROS_WARN("TESTING MODE ACTIVE!");
     testing = true;
     role = (ROLE)(std::stoi(argv[1])-5);
   }

   if(role != DATA_CENTER)
     parent_name = argv[2];
   if(role != EXPLORER)
     child_name = argv[3];

   if(role == RELAY && !testing)
       ros::Duration(20).sleep();

    
   std::string robot_name = ros::this_node::getNamespace();
   ROS_INFO_STREAM(robot_name);
   robot_name.erase(robot_name.begin());
   ROS_INFO_STREAM(robot_name);

   TeamScheduler team(nh, robot_name, role, parent_name, child_name, testing);
   
   team.exec();

   return 0;
}
