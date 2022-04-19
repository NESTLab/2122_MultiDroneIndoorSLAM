#include <robot_state_machine.h>

float RobotState::meet_loc_x = 0.0;
float RobotState::meet_loc_y = 0.0;
float RobotState::explore_loc_x = 0.0;
float RobotState::explore_loc_y = 0.0;
float RobotState::time_for_exploration = 45.0;
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
  
  geometry_msgs::Point current_pose = explore_interface->getRobotCurrentPose().pose.position;
  meet_loc_x = current_pose.x+3;
  meet_loc_y = current_pose.y-0.5;

  data_dump_location.x = -6;
  data_dump_location.y = -5;

  time_for_exploration = MIN_TIME_FOR_EXPLORATION;

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
  publishRobotState();
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
   geometry_msgs::Point nav_point = getMeetingPoint();
   explore_interface->goToPoint(nav_point);
   send_once = false;
  }
  std::string message = getFormattedMessage("Robot Going to: ");
  ROS_INFO_STREAM(message<<getMeetingPoint());
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
   this_state = true;
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

  std::string message = getFormattedMessage("Robot Going to: ");
  ROS_INFO_STREAM(message<<getMeetingPoint());
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
   geometry_msgs::Point nav_point = getMeetingPoint();
   explore_interface->goToPoint(nav_point);
   send_once = false;
}

void GoToMeet::exitPoint()
{
   this_state = false;
   send_once = false;
   connected = false;
   connection_request_received = false;
   explore_interface->stopRobot();

   last_robot_state = (TEAM_STATES)(m_unId);
   printMessage("Exiting the state GO_TO_MEET");
}


void GoToMeet::connCB(const mdis_state_machine::Connection::ConstPtr msg)
{
  if(!this_state)
    return;

  if(getRobotOfInterestName() == (msg->connection_to.data))
  {
    connected = true;
    connected_robot_name = msg->connection_to.data;
    time_of_last_conn = ros::Time::now();
  }
}

void GoToMeet::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  if(!this_state)
    return;
    
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
   printMessage("Entrypoint for TransitToMeet");
   connection_request_received = false;
   current_publishing_counter = 0;
   this_state = true;
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

   if(current_publishing_counter > MAX_PUBLISH_COUNT)
      return ERROR_STATE;
    
   ROS_WARN_STREAM(current_publishing_counter);
   
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
  this_state = false;
  printMessage("Exitpoint for TransitToMeet");
}


void TransitToMeet::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  if(!this_state)
    return;
    
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
    {
      return DECIDE_NEXT_MEETING;
    }

    else if(robot_role == RELAY)
    {
      if (connected_robot_name == data_center_name)
        return END_MEETING;
      else
        return RECEIVE_NEXT_MEETING;
    }

    else
    {
        return END_MEETING;
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
   frontier_received = false;
   this_state = true;
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
  if(!this_state)
    return;
    
   for (int i=0;i<msg->poses.size();i++){
     frontiers_list.push_back(msg->poses.at(i).position);
  }
  frontier_received = true;
}

void DecideNextMeeting::updateNextMeetingPoint()
{
  float min_dis = 1000.0;
  geometry_msgs::Point point;
  geometry_msgs::Point temp_pos;
  geometry_msgs::Point best_frontier_location;
  best_frontier_location.x = frontiers_list.at(0).x;
  best_frontier_location.y = frontiers_list.at(0).y;
  setGoToExplorePoint(best_frontier_location);
  for (auto& frontier : frontiers_list)
  {
    //euclidean distance to frontier from explorer meeting point
    geometry_msgs::Point temp;
    geometry_msgs::Point other_robot_location;
    
    
    temp.x=frontier.x;
    temp.y=frontier.y;
    float total_distance_from_frontier = 0;
    
    // float self_distance_from_frontier = explore_interface->getDistancePrediction(temp); 
    float self_distance_from_frontier = explore_interface->getDistancePrediction(frontiers_list.at(0), temp); //experimenting new method

    ROS_ERROR_STREAM("Analyzing frontier point:   "<<frontier);
    // float other_distance_from_frontier = 0.0;
    float other_distance_from_frontier = explore_interface->getDistancePrediction(data_dump_location, temp);
    /* This might be redundant 
    **  The relay will go to the data center and then will come to meet. 
    **  Hence, this fartherst logic does not make much sense in that context. 
    ** 
    */
    // float other_distance_from_frontier = explore_interface->getDistancePrediction(other_robot_location, temp);
    
    total_distance_from_frontier = self_distance_from_frontier + other_distance_from_frontier;
  
    if (total_distance_from_frontier<=min_dis){
      min_dis = total_distance_from_frontier;
      point.x=frontier.x;
      point.y=frontier.y;
      setMeetingPoint(point);      
    }
  }
}

void DecideNextMeeting::exitPoint()
{
   printMessage("Exitpoint for DecideNextMeeting");
   updated_meeting_location = false;
   this_state = false;
   frontier_received = false;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// I D L E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ReceiveNextMeeting::entryPoint()
{
   connection_request_received = false;
   this_state = true;
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
   this_state = false;
  last_robot_state = (TEAM_STATES)(m_unId);
  printMessage("Exitpoint for ReceiveNextMeeting");
}


void ReceiveNextMeeting::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  if(!this_state)
    return;
    
  std::string conn_robot_name = msg->robot_name.data;
  std::string conn_robot_request_name = msg->connection_to.data;
  if (connected_robot_name == (conn_robot_name) && conn_robot_request_name == robot_name)
  {
      setMeetingPoint(msg->next_meeting_point);
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
   this_state = true;
   meeting_in_time = 0;
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
      {
        return GO_TO_EXPLORE;
      }

      else if(robot_role == RELAY)
      {
        if (connected_robot_name == data_center_name)
          return GO_TO_MEET;
        else        
          return GO_TO_DUMP_DATA;
      }

      else
      {
        return DATA_CENTER_READY_TO_MEET;
      }
   }
   
   return END_MEETING;
}

void EndMeeting::step()
{
  if (robot_role == RELAY && meeting_in_time==0)
    calculateTimeForMeeting();

  if (robot_role == EXPLORER && meeting_in_time != 0)
    calculateTimeForExploration();

  printMessageThrottled("Step for EndMeeting");
  publishRobotState();
  publishConnectionRequest();
  current_publishing_counter++;
}

void EndMeeting::calculateTimeForMeeting()
{
    geometry_msgs::Point meeting_point = getMeetingPoint();

    float time_to_go_home = explore_interface->getTimePredictionForTravel(data_dump_location);
    float time_to_meet_after_home = explore_interface->getTimePredictionForTravel(data_dump_location, meeting_point);

    meeting_in_time = time_to_go_home + time_to_meet_after_home;
}

void EndMeeting::calculateTimeForExploration()
{
    geometry_msgs::Point meeting_point = getMeetingPoint();
    float time_to_reach_exploration_site = explore_interface->getTimePredictionForTravel(meeting_point);
    float expected_exploration_time = meeting_in_time - time_to_reach_exploration_site;
    
    time_for_exploration = expected_exploration_time > MIN_TIME_FOR_EXPLORATION ? expected_exploration_time : MIN_TIME_FOR_EXPLORATION;
}

void EndMeeting::exitPoint()
{
  last_robot_state = (TEAM_STATES)(m_unId);
  printMessage("Exitpoint for EndMeeting");
  connected_robot_name = "";
  meeting_in_time = 0;
   this_state = false;
}


void EndMeeting::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  if(!this_state)
    return;
    
  std::string conn_robot_name = msg->robot_name.data;
  std::string conn_robot_request_name = msg->connection_to.data;
  if (connected_robot_name == (conn_robot_name) && conn_robot_request_name == robot_name)
  {
    if(msg->robot_state == END_MEETING)
    {
        connection_request_received = true;
        if(robot_role == EXPLORER)
            meeting_in_time = msg->meeting_in_time;
    }
  }
}

void EndMeeting::publishConnectionRequest()
{
   mdis_state_machine::ConnectionRequest data;
   data.robot_name.data = robot_name;
   data.connection_to.data = connected_robot_name;
   data.robot_state = (int)(m_unId);
   if(robot_role == EXPLORER)
      data.next_meeting_point = getMeetingPoint();
   if(robot_role == RELAY)
      data.meeting_in_time = meeting_in_time;

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
   this_state = true;
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
  printMessageThrottled("Executing the step for GO_TO_DUMP_DATA");
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
   explore_interface->goToPoint(data_dump_location);
   send_once = false;
}

void GoToDumpData::exitPoint()
{
   send_once = false;
   connected = false;
   connection_request_received = false;
   this_state = false;
   explore_interface->stopRobot();

   last_robot_state = (TEAM_STATES)(m_unId);
   printMessage("Exiting the state GO_TO_DUMP_DATA");
}


void GoToDumpData::connCB(const mdis_state_machine::Connection::ConstPtr msg)
{
  if(!this_state)
    return;
    
  if(getRobotOfInterestName() == (msg->connection_to.data))
  {
    connected = true;
    connected_robot_name = msg->connection_to.data;
    time_of_last_conn = ros::Time::now();
  }
}

void GoToDumpData::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  if(!this_state)
    return;
    
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
///////////////////////////////////// D.  C.   R E A D Y   T O   M E E T   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool DataCenterReadyToMeet::entryPoint()
{
   printMessage("Entering the state DATA_CENTER_READY_TO_MEET");
   connection_request_received = false;
   downtime_counter = 0;
   this_state = true;
   return true;
}

bool DataCenterReadyToMeet::isDone()
{
  if(downtime_counter++>MAX_DOWNTIME)
     return connection_request_received;
  
  return false;
}


TEAM_STATES DataCenterReadyToMeet::transition()
{
   if(isDone())
      return TRANSIT_TO_MEET;
   
   return DATA_CENTER_READY_TO_MEET;
}

void DataCenterReadyToMeet::step()
{

  publishRobotState();
  printMessageThrottled("Executing the step for DATA_CENTER_READY_TO_MEET");
}

void DataCenterReadyToMeet::exitPoint()
{
   connection_request_received = false;
   this_state = false;
   explore_interface->stopRobot();

   last_robot_state = (TEAM_STATES)(m_unId);
   printMessage("Exiting the state DATA_CENTER_READY_TO_MEET");
}

void DataCenterReadyToMeet::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  if(!this_state)
    return;
    
  if(connection_request_received)
      return;

  if(downtime_counter <= MAX_DOWNTIME)
     return;

  std::string conn_robot_name = msg->robot_name.data;
  std::string conn_robot_request_name = msg->connection_to.data;
  if (conn_robot_request_name == robot_name){
    connection_request_received = true;
    connected_robot_name = conn_robot_name;
  }
}

std::string DataCenterReadyToMeet::getRobotOfInterestName()
{
  return parent_robot_name;  
}
