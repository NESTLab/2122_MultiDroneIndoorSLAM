#include <robot_state_machine.h>

float RobotState::curr_meet_x = 0.0;
float RobotState::curr_meet_y = 0.0;
float RobotState::next_meet_x = 0.0;
float RobotState::next_meet_y = 0.0;
float RobotState::time_for_exploration = 20.0;
std::string RobotState::parent_robot_name = "";
std::string RobotState::child_robot_name = "";
ROLE RobotState::robot_role;

RobotState::RobotState(uint64_t un_id, const std::string& str_name, ros::NodeHandle &nh) :
      // m_pcTeam(nullptr), 
      m_unId(un_id), m_strName(str_name) 
{
  explore_interface = new MoveBaseInterface(nh);
  robot_name = ros::this_node::getNamespace();
  robot_name.erase(robot_name.begin());
  curr_meet_x = -31;
  curr_meet_y = -6;
  next_meet_x = -28;
  next_meet_y = -6;
  data_dump_location.x = -31;
  data_dump_location.y = -10;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// I D L E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Idle::entryPoint()
{
   explore_interface->stopRobot();
  //  ROS_INFO("IDLING IDLING IDLING IDLING");
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
  ROS_INFO("GOING TO EXPLORE!!!!!!!!!!!!!!!!!!!!!");
   return true;
}

bool GoToExplore::isDone()
{
   ROS_INFO_STREAM("Going to explore at"<<getNextMeetingPoint());
   geometry_msgs::Point temp_point = getNextMeetingPoint();
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
  ROS_INFO("Step for GoToExplore");
}

void GoToExplore::exitPoint() 
{
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// E X P L O R E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Explore::entryPoint()
{
  ROS_INFO("EXPLORE!!!!!!!!!!!!!!!!!!!!!");
   starting_time = ros::Time::now();
   ROS_INFO_STREAM("Exploring for "<<time_for_exploration<<" Seconds");
   return true;
}

bool Explore::isDone()
{
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
  ROS_INFO_THROTTLE(10,"Step for Explore");
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
  // setNextMeetingPoint(explore_interface->getRobotCurrentPose().pose.position); //remove, meeting decided during meeting now
  
  ROS_INFO_STREAM("Next meeting"<<getNextMeetingPoint());
  // ROS_INFO_STREAM("Next meeting"<<getCurrentMeetingPoint());

  explore_interface->stopRobot();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O   T O   M E E T   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToMeet::entryPoint()
{
   ROS_INFO_STREAM("Going to meet at"<<getCurrentMeetingPoint());
   geometry_msgs::Point temp_point = getCurrentMeetingPoint();
   explore_interface->goToPoint(temp_point, false);
   ros::Duration(1).sleep();  // Giving it some time to reflect
   return true;
}

bool GoToMeet::isDone()
{
   if(connected)
   {
     if(isConnDirectRelated(conn_robot))
     {
       ROS_INFO("Robot is connected to the party of interest");
       return true;
     }
   }
   return false;
}


TEAM_STATES GoToMeet::transition() 
{
  if(isDone())
  {
    return MEET;
  }
  else 
    return GO_TO_MEET;
}

void GoToMeet::step()
{
  ROS_INFO_THROTTLE(10,"Step for GoToMeet");
}

void GoToMeet::exitPoint() 
{
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// M E E T   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Meet::entryPoint()
{
  // ros::Duration(5).sleep();
   data_received = false;
   frontier_data_received = false;
   location_data_received = false;
   location_data_ack = false;
   return true;
}

bool Meet::isDone()
{
   // Data sync done
   return true;
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
  else 
    return MEET;
}

void Meet::step()
{
  ROS_INFO("Step for Meet");
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
    ros::Duration(0.1).sleep();
    location_data_ack_pub.publish(data);
  }
  ROS_INFO("Exiting");
  geometry_msgs::Point temp_location = explore_interface->getRobotCurrentPose().pose.position;
  robot_locations_during_meeting.push_back(temp_location);
  robot_locations_during_meeting.push_back(location_msg);

  // Set this only if the meeting was successful
  if(robot_role == EXPLORER)
  {
    publishNextMeetingLocation();
    setExplorationTime();
    setCurrAsNextMeeting();
  }
  else
    getNextMeetingLocationFromCallback();
  
  robot_locations_during_meeting.clear();
  


  
    

  // get time estimate from move_base_interface
  // set exploration_duration accordingly
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
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  mdis_state_machine::DataCommunication data;
  data.connection_between.resize(2);
  data.connection_between.at(0).data = robot_name;
  data.connection_between.at(1).data = parent_robot_name;
  // ROS_INFO_STREAM("DATADTATDTADTTADTATDATDATADATDATDTADTATDATDATDTAATDATDTATDATA"<<robot_locations_during_meeting.size());
  data.next_meeting_point = getNextFurthestMeetingPoint(frontier_msg, robot_locations_during_meeting);
  
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
  while(ros::ok() && !data_received)
  {
    ROS_INFO("Waiting for meeting data");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("Data received");
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
   return true;
}

bool GoToDumpData::isDone()
{
   ROS_INFO_STREAM("Going to Dump Data at"<<data_dump_location);
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
  explore_interface->stopRobot();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D U M P   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool DumpData::entryPoint()
{
   return true;
}

bool DumpData::isDone()
{
   // Data sync done
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
}
