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
  if(!testing)
  {
    explore_interface = new MoveBaseInterface(nh);
  }
  else
  {
    ROS_INFO("Testing mode, skipping initializations");
    RobotState::testing_mode = true;
  }
  robot_name = ros::this_node::getNamespace();
  robot_name.erase(robot_name.begin());
  
  int robot_num = (int)(robot_name.back())-48;
  geometry_msgs::Point current_pose = explore_interface->getRobotCurrentPose().pose.position;
  curr_meet_x = current_pose.x;
  curr_meet_y = current_pose.y;
  next_meet_x = current_pose.x;
  next_meet_y = current_pose.y;

  data_dump_location.x = 0;
  data_dump_location.y = 0;
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
  ROS_INFO("Step for Idle");
}

void Idle::exitPoint()
{
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O   T O   E X P L O R E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToExplore::entryPoint()
{
   ROS_INFO("Entering the state GO_TO_EXPLORE");
   return true;
}

bool GoToExplore::isDone()
{
   ROS_INFO_STREAM("Going to explore at "<<getNextMeetingPoint());
   geometry_msgs::Point temp_point = getNextMeetingPoint();

   state_pub_data.robot_name.data = robot_name;
   state_pub_data.robot_state = (int)GO_TO_EXPLORE;

   if(RobotState::testing_mode)
   {
      int i = 0;
      int rate = 5;
      
      // Hardcoded publish for 5 seconds
      while(i++ < testing_waiting_time*rate)
      {
        ros::Rate(rate).sleep();
        robot_state_pub.publish(state_pub_data);
      }

      return true;
   }
  
   robot_state_pub.publish(state_pub_data);
   explore_interface->goToPoint(temp_point, true);
   return true;
}

TEAM_STATES GoToExplore::transition()
{
  if(isDone())
    return EXPLORE;
  else
    return GO_TO_EXPLORE;
}

void GoToExplore::step()
{
  ROS_INFO("Executing step for GO_TO_EXPLORE");
}

void GoToExplore::exitPoint()
{
   ROS_INFO("Exiting the state GO_TO_EXPLORE");
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// E X P L O R E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Explore::entryPoint()
{
   starting_time = ros::Time::now();
   ROS_INFO("Entering the state EXPLORE");
   ROS_INFO_STREAM("Exploring for "<<time_for_exploration<<" Seconds");
   return true;
}

bool Explore::isDone()
{
   state_pub_data.robot_name.data = robot_name;
   state_pub_data.robot_state = (int)EXPLORE;

   if(RobotState::testing_mode)
   {
      int i = 0;
      int rate = 5;
      
      // Hardcoded publish for 5 seconds
      while(i++ < testing_waiting_time*rate)
      {
        ros::Rate(rate).sleep();
        robot_state_pub.publish(state_pub_data);
      }
      return true;
   }
  
   ros::Duration time_since_start = ros::Time::now() - starting_time;
   ros::Duration time_until_next_meeting = ros::Duration(time_for_exploration);
   return time_since_start > time_until_next_meeting;
}

TEAM_STATES Explore::transition()
{
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
  robot_state_pub.publish(state_pub_data);
  ROS_INFO_THROTTLE(10,"Executing step for EXPLORE");
}

void Explore::exitPoint()
{
  std_msgs::Bool msg;
  msg.data = true;
  pause_exploration_pub.publish(msg);

  // TO-DO
  // BUG!!!!
  // Values are being changed only for the class Explore
  // Need to change values for all
  if(!RobotState::testing_mode)
  {
     setNextMeetingPoint(explore_interface->getRobotCurrentPose().pose.position);
     explore_interface->stopRobot();
  }
  
  ROS_INFO_STREAM("Exiting the state EXPLORE\nNext meeting "<<getNextMeetingPoint());

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O   T O   M E E T   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToMeet::entryPoint()
{
   ROS_INFO("Entering the state GO_TO_MEET");
   ROS_INFO_STREAM("Going to meet at"<<getCurrentMeetingPoint());
   geometry_msgs::Point temp_point = getCurrentMeetingPoint();
   // explore_interface->goToPoint(temp_point, false);
   interested = false;
   connected = false;
   if(!RobotState::testing_mode)
     explore_interface->goToPoint(temp_point, false);
   // explore_interface->goToPoint(temp_point, false);
   ros::Duration(1).sleep();  // Giving it some time to reflect
   return true;
}

bool GoToMeet::isDone()
{
   if(connected)
   {
     if(isConnDirectRelated(conn_robot))
     {
       mdis_state_machine::Interest data;
       data.req.resize(2);
       data.req.at(0).data = "Request for connection. Interested?";
       data.req.at(1).data = robot_name;
       interest_pub.publish(data);
       ros::Duration(1).sleep();
       if(interested){
        ROS_INFO("Robot is connected to the party of interest");
        return true;
       }

     }
   }
   return false;
}


TEAM_STATES GoToMeet::transition()
{
  if(isDone())
  {
    connected_robot_name = conn_robot;
    return MEET;
  }
  else
    return GO_TO_MEET;
}

void GoToMeet::step()
{
  state_pub_data.robot_name.data = robot_name;
  state_pub_data.robot_state = (int)GO_TO_MEET;
  robot_state_pub.publish(state_pub_data);

  ROS_INFO_THROTTLE(10,"Executing the step for GO_TO_MEET");
}

void GoToMeet::exitPoint()
{
  ROS_INFO("Exiting the state GO_TO_MEET");
  if(!RobotState::testing_mode)
    explore_interface->stopRobot();
}


void GoToMeet::connCB(const mdis_state_machine::Connection::ConstPtr msg)
{
  connected = false;
  for(int i = 0; i<msg->connection_between.size(); i++)
  {
    if(robot_name == msg->connection_between.at(i).data)
    {
      int j = 1 ? i==0 : 0;
      connected = true;
      conn_robot = msg->connection_between.at(j).data;
    }
  }
}

void GoToMeet::interestCB(const mdis_state_machine::Interest::ConstPtr msg){
  std::string r_name = msg->req.at(1).data;
  ROS_INFO_STREAM("CONNN_ROBOT"<<r_name);
  ROS_INFO_STREAM("ROBOT_NAME"<<robot_name);
  if (robot_name!=r_name){

    ROS_INFO_STREAM("Setting interested as true");
    interested = true;
    ros::spinOnce();
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// M E E T   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Meet::entryPoint()
{
  // ros::Duration(5).sleep();
   ROS_INFO("Entering the state MEET");
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
        robot_state_pub.publish(state_pub_data);
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
  state_pub_data.robot_name.data = robot_name;
  state_pub_data.robot_state = (int)MEET;
  robot_state_pub.publish(state_pub_data);
  
  ROS_INFO("Executing the step for MEET");
  requestMerge(connected_robot_name);
}

void Meet::exitPoint()
{
  if (robot_role == RELAY){
    while(ros::ok() && !location_data_ack){
      ROS_INFO("sending location data");
      ros::spinOnce();
      ROS_INFO_STREAM("bool"<<location_data_ack);
      geometry_msgs::Point temp_location = explore_interface->getRobotCurrentPose().pose.position;
      location_data_pub.publish(temp_location);
      ros::Duration(0.5).sleep();
    }

  }
  if (robot_role == EXPLORER){
    while(ros::ok() && !location_data_received)
  {
    ROS_INFO("Waiting for location data");
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
  ROS_INFO("Exiting the state MEET");
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

  ROS_INFO("Attemping merge...");
  if(mergeRequestClient.call(mergeRequest))
  {
    success = mergeRequest.response.success;
    if(success)
    {
      ROS_INFO_STREAM("Successful merge with: " << conn_robot);
    }
    else
    {
      ROS_INFO_STREAM("Failed merge with: " << conn_robot);
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
    ROS_INFO("Waiting for frontier data");
    std_msgs::String data;
    data.data = robot_name;
    // ROS_INFO_STREAM("ROBOT NAME IS"<<robot_name);
    frontier_req_pub.publish(data);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  mdis_state_machine::DataCommunication data;
  data.connection_between.resize(2);
  data.connection_between.at(0).data = robot_name;
  data.connection_between.at(1).data = parent_robot_name;
  // ROS_INFO_STREAM("DATADTATDTADTTADTATDATDATADATDATDTADTATDATDATDTAATDATDTATDATA"<<robot_locations_during_meeting.size());
  data.next_meeting_point = getNextFurthestMeetingPoint(frontier_msg, robot_locations_during_meeting);
  ROS_INFO_STREAM("NEXT MEETING POINT"<<data.next_meeting_point);
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
    ROS_INFO("Waiting for meeting data");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("Data received");
  setCurrentMeetingPoint(buffer_next_location);
  ros::Duration(0.1).sleep();
  setCurrentMeetingPoint(buffer_next_location);
}

void Meet::nextMeetingLocationCB(const mdis_state_machine::DataCommunication::ConstPtr msg)
{
  ROS_INFO("Data Received");
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
   ROS_INFO("Entering the state GO_TO_DUMP");
   return true;
}

bool GoToDumpData::isDone()
{
   ROS_INFO_STREAM("Going to Dump Data at"<<data_dump_location);
 
   state_pub_data.robot_name.data = robot_name;
   state_pub_data.robot_state = (int)GO_TO_DUMP_DATA;

   if(RobotState::testing_mode)
   {
      int i = 0;
      int rate = 5;
      
      // Hardcoded publish for 5 seconds
      while(i++ < testing_waiting_time*rate)
      {
        ros::Rate(rate).sleep();
        robot_state_pub.publish(state_pub_data);
      }

      return true;
   }
   robot_state_pub.publish(state_pub_data);
   explore_interface->goToPoint(data_dump_location, true);
   return true;
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
}

void GoToDumpData::exitPoint()
{
  ROS_INFO("Exiting the state GO_TO_DUMP");
  if(!RobotState::testing_mode)
    explore_interface->stopRobot();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D U M P   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool DumpData::entryPoint()
{
   ROS_INFO("Entering the state DUMP_DATA");
   return true;
}

bool DumpData::isDone()
{
   // Data sync done

   state_pub_data.robot_name.data = robot_name;
   state_pub_data.robot_state = (int)DUMP_DATA;

   if(RobotState::testing_mode)
   {
      int i = 0;
      int rate = 5;
      
      // Hardcoded publish for 5 seconds
      while(i++ < testing_waiting_time*rate)
      {
        ros::Rate(rate).sleep();
        robot_state_pub.publish(state_pub_data);
      }

      return true;
   }
   robot_state_pub.publish(state_pub_data);
  
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
  ROS_INFO("Step for DumpData");
}

void DumpData::exitPoint()
{
    ROS_INFO("Exiting the state DUMP_DATA");
}
