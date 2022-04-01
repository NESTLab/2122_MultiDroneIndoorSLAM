#include <robot_state_machine.h>

float RobotState::curr_meet_x = 0.0;
float RobotState::curr_meet_y = 0.0;
float RobotState::next_meet_x = 0.0;
float RobotState::next_meet_y = 0.0;
float RobotState::time_for_exploration = 40.0;

std::string RobotState::parent_robot_name = "";
std::string RobotState::child_robot_name = "";
std::string connected_robot_name = "";

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
  ROS_INFO_STREAM("["<<robot_name<<"] "<< "Step for Idle");
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
  printMessage("Executing step for GO_TO_EXPLORE");
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
   if (connection_request_received)
      return MEET;

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
   data.connection_to.data = conn_robot;
   data.robot_state = (int)(m_unId);
   connection_request_pub.publish(data);
}

void GoToMeet::sendRobotToLocation()
{
   geometry_msgs::Point nav_point = getCurrentMeetingPoint();
   ROS_INFO_STREAM("Current Meeting point:"<<nav_point);
   explore_interface->goToPoint(nav_point);
   send_once = false;
}

void GoToMeet::exitPoint()
{
   send_once = false;
   connected = false;
   connection_request_received = false;
   explore_interface->stopRobot();

   printMessage("Exiting the state GO_TO_MEET");
}


void GoToMeet::connCB(const mdis_state_machine::Connection::ConstPtr msg)
{
  if(isConnDirectRelated(msg->connection_to.data))
  {
    connected = true;
    conn_robot = msg->connection_to.data;
    time_of_last_conn = ros::Time::now();
  }
}

void GoToMeet::connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg)
{
  std::string conn_robot_name = msg->robot_name.data;
  std::string conn_robot_request_name = msg->connection_to.data;
  if (isConnDirectRelated(conn_robot_name) && conn_robot_request_name == robot_name){
    connection_request_received = true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// M E E T   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Meet::entryPoint()
{
  // ros::Duration(5).sleep();
   ROS_INFO_STREAM("["<<robot_name<<"] "<< "Entering the state MEET");
   data_received = false;
   frontier_data_received = false;
   location_data_received = false;
   location_data_ack = false;
   is_merge_complete = false;

   return true;
}

bool Meet::isDone()
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
   }
   return is_merge_complete;
}

TEAM_STATES Meet::transition()
{
  if(isDone())
  {
    if (robot_role == EXPLORER)
      return GO_TO_EXPLORE;
    else if(robot_role == RELAY)
      return GO_TO_DUMP_DATA;
    // else if(robot_role == RELAY_BETN_ROBOTS)
    //   return GO_TO_MEET;
  }
  return MEET;
}

void Meet::step()
{
  publishRobotState();
  ROS_INFO_STREAM("["<<robot_name<<"] "<< "Executing the step for MEET");
  requestMerge(connected_robot_name);
}

void Meet::exitPoint()
{
  if (robot_role == RELAY){
    while(ros::ok() && !location_data_ack){
      ROS_INFO_STREAM("["<<robot_name<<"] "<< "sending location data");
      ros::spinOnce();
      ROS_INFO_STREAM("["<<robot_name<<"] "<<"bool"<<location_data_ack);
      geometry_msgs::Point temp_location = explore_interface->getRobotCurrentPose().pose.position;
      location_data_pub.publish(temp_location);
      ros::Duration(0.5).sleep();
    }

  }
  if (robot_role == EXPLORER){
    while(ros::ok() && !location_data_received)
  {
    ROS_INFO_STREAM("["<<robot_name<<"] "<< "Waiting for location data");
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }
    mdis_state_machine::LocationAck data;
    data.ack.resize(1);
    data.ack.at(0).data = "Received";
    // std::string temp_msg = "Received";
    location_data_ack_pub.publish(data);
    // ros::Duration(0.1).sleep();
    // location_data_ack_pub.publish(data);
  }
  ROS_INFO_STREAM("["<<robot_name<<"] "<< "Exiting the state MEET");
  geometry_msgs::Point temp_location = explore_interface->getRobotCurrentPose().pose.position;
  robot_locations_during_meeting.push_back(temp_location);
  robot_locations_during_meeting.push_back(location_msg);

  // Set this only if the meeting was successful
  if(robot_role == EXPLORER)
  {
    publishNextMeetingLocation();
    if(!RobotState::testing_mode)
      setExplorationTime();
    setCurrAsNextMeeting();
  }
  else
    getNextMeetingLocationFromCallback();

  robot_locations_during_meeting.clear();

  // get time estimate from move_base_interface
  // set exploration_duration accordingly
}

void Meet::requestMerge(std::string conn_robot)
{
  coms::TriggerMerge mergeRequest;
  mergeRequest.request.robot_id = conn_robot;
  bool success = false;

  ROS_INFO_STREAM("["<<robot_name<<"] "<< "Attemping merge...");
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

void Meet::setExplorationTime()
{
  geometry_msgs::Point curr_location = explore_interface->getRobotCurrentPose().pose.position;
  geometry_msgs::Point temp_next_location = getNextMeetingPoint();
  float time_to_dump = explore_interface->getTimePredictionForTravel(curr_location, data_dump_location);
  float time_to_meet_after = explore_interface->getTimePredictionForTravel(data_dump_location, temp_next_location);
  float time_to_exploration_site = explore_interface->getTimePredictionForTravel(curr_location, temp_next_location);
  float total_time = time_to_dump+time_to_meet_after;

  time_for_exploration = (total_time-time_to_exploration_site)/2;
}

void Meet::getFrontiersCB(const explore_lite::FrontiersArray::ConstPtr msg)
{
   for (int i=0;i<msg->frontiers.size();i++){
     frontier_msg.push_back(msg->frontiers.at(i));
  }
  frontier_data_received = true;
}

void Meet::getBestFrontiersCB(const geometry_msgs::PoseArray::ConstPtr msg)
{
   for (int i=0;i<msg->poses.size();i++){
     frontier_msg.push_back(msg->poses.at(i).position);
  }
  frontier_data_received = true;
}

void Meet::getLocationCB(const mdis_state_machine::Location::ConstPtr msg){
  // geometry_msgs::Point location_msg;
  location_msg=msg->robot_location;
  location_data_received = true;
}

void Meet::getLocationAckCB(const mdis_state_machine::LocationAck::ConstPtr msg){
  location_data_ack = true;
}

void Meet::publishNextMeetingLocation()
{
    while(ros::ok() && !frontier_data_received)
  {
    ROS_INFO_STREAM("["<<robot_name<<"] "<< "Waiting for frontier data");
    std_msgs::String data;
    data.data = robot_name;
    // ROS_INFO_STREAM("["<<robot_name<<"] "<<"ROBOT NAME IS"<<robot_name);
    frontier_req_pub.publish(data);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  mdis_state_machine::DataCommunication data;
  data.connection_between.resize(2);
  data.connection_between.at(0).data = robot_name;
  data.connection_between.at(1).data = parent_robot_name;
  // ROS_INFO_STREAM("["<<robot_name<<"] "<<"DATADTATDTADTTADTATDATDATADATDATDTADTATDATDATDTAATDATDTATDATA"<<robot_locations_during_meeting.size());
  data.next_meeting_point = getNextFurthestMeetingPoint(frontier_msg, robot_locations_during_meeting);
  ROS_INFO_STREAM("["<<robot_name<<"] "<<"NEXT MEETING POINT"<<data.next_meeting_point);
  meeting_data_pub.publish(data);
  ros::Duration(0.2).sleep();
  meeting_data_pub.publish(data);
  ros::Duration(0.2).sleep();
  meeting_data_pub.publish(data);
  ros::Duration(0.2).sleep();
}

void Meet::getNextMeetingLocationFromCallback()
{
  // @to-do
  // THIS NEEDS TO EXIT SOMETIME IF NO CONNECTION IS MADE
  while(ros::ok() && (!testing_mode && !data_received))
  {
    ROS_INFO_STREAM("["<<robot_name<<"] "<< "Waiting for meeting data");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO_STREAM("["<<robot_name<<"] "<< "Data received");
  setCurrentMeetingPoint(buffer_next_location);
  ros::Duration(0.1).sleep();
  setCurrentMeetingPoint(buffer_next_location);
}

void Meet::nextMeetingLocationCB(const mdis_state_machine::DataCommunication::ConstPtr msg)
{
  ROS_INFO_STREAM("["<<robot_name<<"] "<< "Data Received");
  std::string conn_robot;
  for(int i = 0; i<msg->connection_between.size(); i++)
  {
    if(robot_name == msg->connection_between.at(i).data)
    {
      int j = 1 ? i==0 : 0;
      conn_robot = msg->connection_between.at(j).data;
    }
  }
  if(isConnDirectRelated(conn_robot))
  {
    buffer_next_location = msg->next_meeting_point;
    data_received = true;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O  T O  D U M P   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToDumpData::entryPoint()
{
   ROS_INFO_STREAM("["<<robot_name<<"] "<< "Entering the state GO_TO_DUMP");
   ROS_INFO_STREAM("["<<robot_name<<"] "<< "Going to Dump Data at"<<data_dump_location);
   connected = false;
   explore_interface->goToPoint(data_dump_location, false);
   ros::Duration(1).sleep();
   return true;
}

bool GoToDumpData::isDone()
{
   ROS_INFO_STREAM("["<<robot_name<<"] "<<"Going to Dump Data at"<<data_dump_location);
 
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
   if(connected){
      ROS_INFO("Dump Connection Made");
      return true;
   }
   return false;
  
  //  explore_interface->goToPoint(data_dump_location, true);
  //  return true;
}

TEAM_STATES GoToDumpData::transition()
{
  if(isDone())
    return DUMP_DATA;
  else
    return GO_TO_DUMP_DATA;
}

void GoToDumpData::step()
{
  ROS_INFO_THROTTLE(10, "Going to dump data");
  ros::Duration time_since_connection = ros::Time::now() - time_of_last_conn;
  if(time_since_connection>wait_time_for_conn)
    connected = false;

}

void GoToDumpData::exitPoint()
{
  ROS_INFO_STREAM("["<<robot_name<<"] "<< "Exiting the state GO_TO_DUMP");
  if(!RobotState::testing_mode)
    explore_interface->stopRobot();
}

void GoToDumpData::connCB(const mdis_state_machine::Connection::ConstPtr msg)
{
  if(msg->connection_to.data == "data_center")
  {
    connected = true;
    time_of_last_conn = ros::Time::now();
  }
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
