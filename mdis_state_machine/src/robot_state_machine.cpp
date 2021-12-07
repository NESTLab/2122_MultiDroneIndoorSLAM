#include <robot_state_machine.h>

float RobotState::curr_meet_x = 0.0;
float RobotState::curr_meet_y = 0.0;
float RobotState::next_meet_x = 0.0;
float RobotState::next_meet_y = 0.0;

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
   starting_time = ros::Time::now();
   return true;
}

bool Explore::isDone()
{
   ros::Duration time_since_start = ros::Time::now() - starting_time;
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
  setNextMeetingLocation(explore_interface->getRobotCurrentPose().pose.position);
  
  ROS_INFO_STREAM("Next meeting"<<getNextMeetingPoint());

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
   // Giving it some time to reflect
   ros::Duration(1).sleep();
   return true;
}

bool GoToMeet::isDone()
{
   if(connected)
   {
     if(isConnDirectRelated())
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
  // Set this only if the meeting was successful
  setCurrAsNextMeeting();

  // get time estimate from move_base_interface
  // set exploration_duration accordingly
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
