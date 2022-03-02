#include <team_scheduler.h>

TeamScheduler::TeamScheduler(ros::NodeHandle &nh, ROLE role, const std::string& parent_name, const std::string& child_name, bool testing)
{
    addStates(nh, testing);
    if (role == EXPLORER)
      setInitialState(GO_TO_EXPLORE);
    else if (role == RELAY)
      setInitialState(GO_TO_MEET);
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

void TeamScheduler::addStates(ros::NodeHandle &nh, bool testing_mode)
{
    addState(new Idle(nh, testing_mode));
    addState(new GoToExplore(nh, testing_mode));
    addState(new Explore(nh, testing_mode));
    addState(new GoToMeet(nh, testing_mode));
    addState(new Meet(nh, testing_mode));
    addState(new GoToDumpData(nh, testing_mode));
    addState(new DumpData(nh, testing_mode));
}

void TeamScheduler::addState(RobotState* pc_state) {
   if(MACRO_STATE_PTR_MAP.find(pc_state->getId()) == MACRO_STATE_PTR_MAP.end()) {
      MACRO_STATE_PTR_MAP[pc_state->getId()] = pc_state;
      // pc_state->setTeam(*this);
   }
   else {
      ROS_ERROR_STREAM("[mdis_state_machine | team_scheduler.cpp ]: Duplicated state id :" << pc_state->getId());
   }
}

// TEAM_STATES RobotState::getState(uint64_t un_state) {
//    return m_pcTeam->getState(un_state);
// }

RobotState& TeamScheduler::getStatePtr(uint64_t un_id)
{
   auto pcState = MACRO_STATE_PTR_MAP.find(un_id);
   if(pcState != MACRO_STATE_PTR_MAP.end()) {
      return *(pcState->second);
   }
   else {
      ROS_ERROR_STREAM("[TEAM_LEVEL | team_scheduler ]:Can't get state id " << un_id);
   }
}

void TeamScheduler::setInitialState(uint64_t un_state) {
   auto pcState = MACRO_STATE_PTR_MAP.find(un_state);
   // if state exists in map, then set it to the initial state of the scheduler
   if(pcState != MACRO_STATE_PTR_MAP.end()) {
      // acquire value of the state (every map has a key(first) and a value(second))
      current_state_ptr = pcState->second;

      // completes entry point of the initial state
      current_state_ptr->entryPoint();
      setTeamMacroState((TEAM_STATES) un_state);
   }
   else {
      ROS_ERROR_STREAM("[TEAM_LEVEL | team_scheduler ]: Can't set initial state to " << un_state);
   }
}

void TeamScheduler::step() {
   /* Only execute if 'current' was initialized */
   if(current_state_ptr) {
      //  ROS_INFO_STREAM(hired_scout << "  " << hired_excavator << "  " << hired_hauler);
    //    ROS_INFO_STREAM("Robot enum:" << SCOUT_1);
      /* Attempt a transition, every state of every rover has its own transition() */
      TEAM_STATES newStateEnum = current_state_ptr->transition();
      RobotState* cNewState = &getStatePtr(newStateEnum);
      // RobotState* cNewState = current_state_ptr;
      if (new_state_request)
      {
         cNewState = &getStatePtr(robot_state);
         new_state_request = false;
      }

      if(cNewState != current_state_ptr) {
         /* Perform transition */
         current_state_ptr->exitPoint();
         // cNewState->setResetRobot(reset_robot_odometry);
         bool entry = cNewState->entryPoint();

         setTeamMacroState((TEAM_STATES) cNewState->getId());
         if(!entry)
            return;
         current_state_ptr = cNewState;
      }
      /* Execute current state */
      current_state_ptr->step();
   }
   else {
      ROS_ERROR_STREAM("[TEAM_LEVEL | team_scheduler ]: The Team has not been initialized, you must call SetInitialState()");
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

  //  if (argc != 4 && argc != 2)
  //  {
  //    ROS_ERROR("This node must be launched with number of robots as argument");
  //    return 0;
  //  }

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
     role = std::stoi(argv[1]) == 5 ? EXPLORER : RELAY;
   }

   parent_name = argv[2];
   if(role != EXPLORER)
     child_name = argv[3];

   if(role == RELAY && !testing)
       ros::Duration(20).sleep();

   TeamScheduler team(nh, role, parent_name, child_name, testing);

   team.exec();

   return 0;
}
