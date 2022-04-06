#include <robot_state_machine.h>

float RobotState::curr_meet_x = 0.0;
float RobotState::curr_meet_y = 0.0;
float RobotState::next_meet_x = 0.0;
float RobotState::next_meet_y = 0.0;
float RobotState::time_for_exploration = 40.0;
TEAM_STATES RobotState::last_robot_state = IDLE;

std::string RobotState::parent_robot_name = "";
std::string RobotState::child_robot_name = "";
std::string RobotState::connected_robot_name = "";

bool RobotState::testing_mode = false;
ROLE RobotState::robot_role;

RobotState::RobotState(uint64_t un_id, const std::string& str_name, ros::NodeHandle &nh, bool testing) :
      // m_pcTeam(nullptr),
      m_unId(un_id), m_strName(str_name)
{
  explore_interface = new MoveBaseInterface(nh, testing);
  robot_state_pub = nh.advertise<mdis_state_machine::RobotsState>(nh.getNamespace() + "/robots_state", 1000);     

  robot_name = ros::this_node::getNamespace();
  robot_name.erase(robot_name.begin());
  
  int robot_num = (int)(robot_name.back())-48;
  geometry_msgs::Point current_pose = explore_interface->getRobotCurrentPose().pose.position;
  curr_meet_x = current_pose.x;
  curr_meet_y = current_pose.y;
  next_meet_x = current_pose.x+3;
  next_meet_y = current_pose.y;

  data_dump_location.x = -6;
  data_dump_location.y = -5;

  ros::Duration(0.050).sleep();
  
  if(testing)
    RobotState::testing_mode = true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// I D L E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Idle::entryPoint()
{
   explore_interface->stopRobot();
   return true;
}

bool Idle::isDone()
{
   return true;
}


TEAM_STATES Idle::transition()
{
  return IDLE;
}

void Idle::step()
{
  printMessageThrottled("Step for Idle");
}

void Idle::exitPoint()
{
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// E R R O R    S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ErrorState::entryPoint()
{
   explore_interface->stopRobot();
   printMessage("Entry point for ErrorState");
   return true;
}

bool ErrorState::isDone()
{
   return true;
}


TEAM_STATES ErrorState::transition()
{
  return ERROR_STATE;
}

void ErrorState::step()
{
  twist_msg.angular.z = direction;
  direction *= -1;
  robot_cmd_vel.publish(twist_msg);
  ros::Duration(0.5).sleep();
  printMessage("Step for ErrorState");
}

void ErrorState::exitPoint()
{
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O   T O   E X P L O R E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToExplore::entryPoint()
{
   printMessage("Entering the state GO_TO_EXPLORE");
   send_once = true;
   return true;
}

bool GoToExplore::isDone()
{
  return explore_interface->navigationDone();
}

TEAM_STATES GoToExplore::transition()
{
  // For explorers, this is the initial state. And for the init states, the transition gets checked before the step fucntion. 
  // Hence this flag is required to make sure the state continues when the state is executed for the first time
  if(send_once)
    return GO_TO_EXPLORE;

  if(isDone())
  {
    if(explore_interface->navigationSucceeded())
      return EXPLORE;
    else
      return ERROR_STATE;
  }
  else
    return GO_TO_EXPLORE;
}

void GoToExplore::step()
{
  printMessageThrottled("Executing step for GO_TO_EXPLORE");
  publishRobotState();
  if(send_once)
  {
   geometry_msgs::Point nav_point = getNextMeetingPoint();
   explore_interface->goToPoint(nav_point);
   send_once = false;
  }
}

void GoToExplore::exitPoint()
{
   send_once = false;
   last_robot_state = (TEAM_STATES)(m_unId);
   printMessage("Exiting the state GO_TO_EXPLORE");
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// E X P L O R E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Explore::entryPoint()
{
   starting_time = ros::Time::now();
   printMessage("Entering the state EXPLORE");
   
   return true;
}

bool Explore::isDone()
{
  if(testing_mode)
    return explore_interface->navigationDone();

   ros::Duration time_since_start = ros::Time::now() - starting_time;
   ros::Duration time_until_next_meeting = ros::Duration(time_for_exploration);
   return time_since_start > time_until_next_meeting;
}

TEAM_STATES Explore::transition()
{
  // Cannot determine through state machine whether the robot is actually exploring or not. 
  //    or has the exploration failed. Hence no logic for the same. 

  if(isDone())
    return GO_TO_MEET;
  else
    return EXPLORE;
}

void Explore::step()
{
  std_msgs::Bool msg;
  msg.data = false;
  pause_exploration_pub.publish(msg);
  printMessageThrottled("Executing step for EXPLORE");
  publishRobotState();
}

void Explore::exitPoint()
{
  std_msgs::Bool msg;
  msg.data = true;
  pause_exploration_pub.publish(msg);

  setNextMeetingPoint(explore_interface->getRobotCurrentPose().pose.position);
  explore_interface->stopRobot();
  
  last_robot_state = (TEAM_STATES)(m_unId);
  printMessage("Exiting the state EXPLORE");
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O   T O   M E E T   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToMeet::entryPoint()
{
   printMessage("Entering the state GO_TO_MEET");
   connection_request_received = false;
   send_once = true;
   return true;
}

bool GoToMeet::isDone()
{
   return explore_interface->navigationDone();
}


TEAM_STATES GoToMeet::transition()
{
    // For relays, this is the initial state. And for the init states, the transition gets checked before the step fucntion. 
  // Hence this flag is required to make sure the state continues when the state is executed for the first time
  if(send_once)
    return GO_TO_MEET;

   if (connection_request_received)
      return TRANSIT_TO_MEET;

   if(isDone())
   {
      if(!explore_interface->navigationSucceeded())
         return ERROR_STATE;
   }
   
   return GO_TO_MEET;
}

void GoToMeet::step()
{
   if(send_once)
      sendRobotToLocation();

  ros::Duration time_since_connection = ros::Time::now() - time_of_last_conn;
  if(time_since_connection>wait_time_for_conn)
    connected = false;

   if(connected)
      publishConnectionRequest();

  publishRobotState();
  printMessageThrottled("Executing the step for GO_TO_MEET");
}

void GoToMeet::publishConnectionRequest()
{
   mdis_state_machine::ConnectionRequest data;
   data.robot_name.data = robot_name;
   data.connection_to.data = connected_robot_name;
   data.robot_state = (int)(m_unId);
   connection_request_pub.publish(data);
}

void GoToMeet::sendRobotToLocation()
{
   geometry_msgs::Point nav_point = getCurrentMeetingPoint();
   explore_interface->goToPoint(nav_point);
   send_once = false;
}

void GoToMeet::exitPoint()
{
   send_once = false;
   connected = false;
   connection_request_received = false;
   explore_interface->stopRobot();

   last_robot_state = (TEAM_STATES)(m_unId);
   printMessage("Exiting the state GO_TO_MEET");
}


void GoToMeet::connCB(const mdis_state_machine::Connection::ConstPtr msg)
{
  if(getRobotOfInterestName() == (msg->connection_to.data))
  {
    connected = true;
    connected_robot_name = msg->connection_to.data;
    time_of_last_conn = ros::Time::now();
  }
}

void GoToMeet::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  if(connected_robot_name == "")
      return;

  std::string conn_robot_name = msg->robot_name.data;
  std::string conn_robot_request_name = msg->connection_to.data;
  if (connected_robot_name == (conn_robot_name) && conn_robot_request_name == robot_name){
    connection_request_received = true;
  }
}

std::string GoToMeet::getRobotOfInterestName()
{
   if(robot_role == EXPLORER)
      return parent_robot_name;
   else
      return child_robot_name;
   
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// T R A N S I T   T O   M E E T   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool TransitToMeet::entryPoint()
{
   connection_request_received = false;
   current_publishing_counter = 0;
   return true;
}

bool TransitToMeet::isDone()
{
   if(current_publishing_counter >= LEAST_PUBLISH_COUNT)
      return connection_request_received;
   return false;
}

TEAM_STATES TransitToMeet::transition()
{
   if(isDone())
      return MERGE_MAP;
   
   return TRANSIT_TO_MEET;
}

void TransitToMeet::step()
{
  printMessageThrottled("Step for TransitToMeet");
  publishRobotState();
  publishConnectionRequest();
  current_publishing_counter++;
}

void TransitToMeet::exitPoint()
{
  last_robot_state = (TEAM_STATES)(m_unId);
  printMessage("Exitpoint for TransitToMeet");
}


void TransitToMeet::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  std::string conn_robot_name = msg->robot_name.data;
  std::string conn_robot_request_name = msg->connection_to.data;
  if (connected_robot_name == (conn_robot_name) && conn_robot_request_name == robot_name){
     if(msg->robot_state == TRANSIT_TO_MEET)
         connection_request_received = true;
  }
}

void TransitToMeet::publishConnectionRequest()
{
   mdis_state_machine::ConnectionRequest data;
   data.robot_name.data = robot_name;
   data.connection_to.data = connected_robot_name;
   data.robot_state = (int)(m_unId);
   connection_request_pub.publish(data);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// M E R G E   M A P    C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MergeMap::entryPoint()
{
   printMessage("Entering the state MERGE_MAP");
   return true;
}

bool MergeMap::isDone()
{
   if(testing_mode)
      return explore_interface->navigationDone();

   return is_merge_complete;
}

TEAM_STATES MergeMap::transition()
{
  if(isDone())
  {
    if(robot_role == EXPLORER)
      return DECIDE_NEXT_MEETING;
    else
    {
      if (connected_robot_name == data_center_name)
        return END_MEETING;
      else
        return RECEIVE_NEXT_MEETING;
    }
  }
  return MERGE_MAP;
}

void MergeMap::step()
{
  publishRobotState();
  printMessageThrottled("Executing the step for MERGE_MAP");
  requestMerge(connected_robot_name);
}

void MergeMap::exitPoint()
{
  printMessage("Exitpoint for MERGE_MAP");
}

void MergeMap::requestMerge(std::string conn_robot)
{
  coms::TriggerMerge mergeRequest;
  mergeRequest.request.robot_id = conn_robot;
  bool success = false;

  printMessage("Attemping merge...");
  if(mergeRequestClient.call(mergeRequest))
  {
    success = mergeRequest.response.success;
    if(success)
    {
      ROS_INFO_STREAM("["<<robot_name<<"] "<<"Successful merge with: " << conn_robot);
    }
    else
    {
      ROS_INFO_STREAM("["<<robot_name<<"] "<<"Failed merge with: " << conn_robot);
    }
  }
  else
  {
    ROS_ERROR("Failed to call merge service");
  }
  is_merge_complete = success;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D E C I D E    N E X T    M E E T I N G   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool DecideNextMeeting::entryPoint()
{
   printMessage("Entrypoint for DecideNextMeeting");
   updated_meeting_location = false;
   return true;
}

bool DecideNextMeeting::isDone()
{
   return updated_meeting_location;
}


TEAM_STATES DecideNextMeeting::transition()
{
  if(isDone())
    return END_MEETING;
  return DECIDE_NEXT_MEETING;
}

void DecideNextMeeting::step()
{
  publishRobotState();
  printMessageThrottled("Step for DecideNextMeeting");

  if(!frontier_received)
    requestFrontiers();

  if(!updated_meeting_location && frontier_received)
  {
    updateNextMeetingPoint();
    updated_meeting_location = true;
  }  
}

void DecideNextMeeting::requestFrontiers()
{
    std_msgs::String data;
    data.data = robot_name;
    frontier_req_pub.publish(data);
}

void DecideNextMeeting::getBestFrontiersCB(const geometry_msgs::PoseArray::ConstPtr msg)
{
   for (int i=0;i<msg->poses.size();i++){
     frontiers_list.push_back(msg->poses.at(i).position);
  }
  frontier_received = true;
}

void DecideNextMeeting::updateNextMeetingPoint()
{
  float max_dis = 0.0;
  geometry_msgs::Point point;
  geometry_msgs::Point temp_pos;

  for (auto& frontier : frontiers_list)
  {
    //euclidean distance to frontier from explorer meeting point
    geometry_msgs::Point temp;
    geometry_msgs::Point other_robot_location;
    
    temp.x=frontier.x;
    temp.y=frontier.y;
    float total_distance_from_frontier = 0;
    
    float self_distance_from_frontier = explore_interface->getDistancePrediction(temp);
    
    float other_distance_from_frontier = 0.0;
    /* This might be redundant 
    **  The relay will go to the data center and then will come to meet. 
    **  Hence, this fartherst logic does not make much sense in that context. 
    ** 
    */
    // float other_distance_from_frontier = explore_interface->getDistancePrediction(other_robot_location, temp);
    
    total_distance_from_frontier = self_distance_from_frontier + other_distance_from_frontier;
  
    if (total_distance_from_frontier>max_dis){
      max_dis = total_distance_from_frontier;
      point.x=frontier.x;
      point.y=frontier.y;
      setNextMeetingPoint(point);      
    }
  }
}

void DecideNextMeeting::exitPoint()
{
   printMessage("Exitpoint for DecideNextMeeting");
   updated_meeting_location = false;
   frontier_received = false;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// I D L E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ReceiveNextMeeting::entryPoint()
{
   connection_request_received = false;
   return true;
}

bool ReceiveNextMeeting::isDone()
{
    return connection_request_received;
}

TEAM_STATES ReceiveNextMeeting::transition()
{
   if(isDone())
      return END_MEETING;
   
   return RECEIVE_NEXT_MEETING;
}

void ReceiveNextMeeting::step()
{
  printMessageThrottled("Step for ReceiveNextMeeting");
  publishRobotState();
}

void ReceiveNextMeeting::exitPoint()
{
  last_robot_state = (TEAM_STATES)(m_unId);
  printMessage("Exitpoint for ReceiveNextMeeting");
}


void ReceiveNextMeeting::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  std::string conn_robot_name = msg->robot_name.data;
  std::string conn_robot_request_name = msg->connection_to.data;
  if (connected_robot_name == (conn_robot_name) && conn_robot_request_name == robot_name)
  {
      setNextMeetingPoint(msg->next_meeting_point);
      connection_request_received = true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// E N D    M E E T I N G   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool EndMeeting::entryPoint()
{
   connection_request_received = false;
   current_publishing_counter = 0;
   return true;
}

bool EndMeeting::isDone()
{
   if(current_publishing_counter >= LEAST_PUBLISH_COUNT)
      return connection_request_received;
   return false;
}

TEAM_STATES EndMeeting::transition()
{
   if(isDone())
   {
      if(robot_role == EXPLORER)
        return GO_TO_EXPLORE;
      else
      {
        if (connected_robot_name == data_center_name)
          return GO_TO_MEET;
        else        
          return GO_TO_DUMP_DATA;
      }
   }
   
   return END_MEETING;
}

void EndMeeting::step()
{
  printMessageThrottled("Step for EndMeeting");
  publishRobotState();
  publishConnectionRequest();
  current_publishing_counter++;
}

void EndMeeting::exitPoint()
{
  last_robot_state = (TEAM_STATES)(m_unId);
  printMessage("Exitpoint for EndMeeting");
}


void EndMeeting::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  std::string conn_robot_name = msg->robot_name.data;
  std::string conn_robot_request_name = msg->connection_to.data;
  if (connected_robot_name == (conn_robot_name) && conn_robot_request_name == robot_name)
  {
    if(msg->robot_state == END_MEETING)
        connection_request_received = true;
  }
}

void EndMeeting::publishConnectionRequest()
{
   mdis_state_machine::ConnectionRequest data;
   data.robot_name.data = robot_name;
   data.connection_to.data = connected_robot_name;
   data.robot_state = (int)(m_unId);
   if(robot_role == EXPLORER)
      data.next_meeting_point = getNextMeetingPoint();

   connection_request_pub.publish(data);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O   T O   D U M P   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToDumpData::entryPoint()
{
   printMessage("Entering the state GO_TO_DUMP_DATA");
   connection_request_received = false;
   send_once = true;
   return true;
}

bool GoToDumpData::isDone()
{
   return explore_interface->navigationDone();
}


TEAM_STATES GoToDumpData::transition()
{
   if (connection_request_received)
      return TRANSIT_TO_MEET;

   if(isDone())
   {
      if(!explore_interface->navigationSucceeded())
         return ERROR_STATE;
   }
   
   return GO_TO_DUMP_DATA;
}

void GoToDumpData::step()
{
   if(send_once)
      sendRobotToLocation();

  ros::Duration time_since_connection = ros::Time::now() - time_of_last_conn;
  if(time_since_connection>wait_time_for_conn)
    connected = false;

   if(connected)
      publishConnectionRequest();

  publishRobotState();
  printMessageThrottled("Executing the step for GO_TO_MEET");
}

void GoToDumpData::publishConnectionRequest()
{
   mdis_state_machine::ConnectionRequest data;
   data.robot_name.data = robot_name;
   data.connection_to.data = connected_robot_name;
   data.robot_state = (int)(m_unId);
   connection_request_pub.publish(data);
}

void GoToDumpData::sendRobotToLocation()
{
   geometry_msgs::Point nav_point = getCurrentMeetingPoint();
   explore_interface->goToPoint(nav_point);
   send_once = false;
}

void GoToDumpData::exitPoint()
{
   send_once = false;
   connected = false;
   connection_request_received = false;
   explore_interface->stopRobot();

   last_robot_state = (TEAM_STATES)(m_unId);
   printMessage("Exiting the state GO_TO_MEET");
}


void GoToDumpData::connCB(const mdis_state_machine::Connection::ConstPtr msg)
{
  if(getRobotOfInterestName() == (msg->connection_to.data))
  {
    connected = true;
    connected_robot_name = msg->connection_to.data;
    time_of_last_conn = ros::Time::now();
  }
}

void GoToDumpData::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  if(connected_robot_name == "")
      return;

  std::string conn_robot_name = msg->robot_name.data;
  std::string conn_robot_request_name = msg->connection_to.data;
  if (connected_robot_name == (conn_robot_name) && conn_robot_request_name == robot_name){
    connection_request_received = true;
  }
}

std::string GoToDumpData::getRobotOfInterestName()
{
  return parent_robot_name;  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D U M P   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool DumpData::entryPoint()
{
   ROS_INFO_STREAM("["<<robot_name<<"] "<< "Entering the state DUMP_DATA");
   return true;
}

bool DumpData::isDone()
{
   // Data sync done

   if(RobotState::testing_mode)
   {
      int i = 0;
      int rate = 5;
      
      // Hardcoded publish for 5 seconds
      while(i++ < testing_waiting_time*rate)
      {
        ros::Rate(rate).sleep();
      }

      return true;
   }
  
   return true;
}

TEAM_STATES DumpData::transition()
{
  if(isDone())
    return GO_TO_MEET;
  else
    return DUMP_DATA;
}

void DumpData::step()
{
  ROS_INFO_STREAM("["<<robot_name<<"] "<< "Step for DumpData");
}

void DumpData::exitPoint()
{
    ROS_INFO_STREAM("["<<robot_name<<"] "<< "Exiting the state DUMP_DATA");
}
