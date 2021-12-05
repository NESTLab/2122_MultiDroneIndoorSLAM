#include <robot_state_machine.h>

RobotState::RobotState(uint64_t un_id, const std::string& str_name, ros::NodeHandle &nh) :
      // m_pcTeam(nullptr), 
      m_unId(un_id), m_strName(str_name) 
{
  explore_interface = new MoveBaseInterface(nh);

  current_meeting_location.x = -31;
  current_meeting_location.y = -6;
  next_meeting_location.x = -28;
  next_meeting_location.y = -6;
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
  return GO_TO_EXPLORE;
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
   ROS_INFO_STREAM("Going to explore at"<<next_meeting_location);
   explore_interface->goToPoint(next_meeting_location, true);
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
  next_meeting_location = explore_interface->getRobotCurrentPose().pose.position;
  
  ROS_INFO_STREAM("Next meeting"<<next_meeting_location);

  explore_interface->stopRobot();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O   T O   M E E T   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToMeet::entryPoint()
{
   ROS_INFO_STREAM("Going to meet at"<<current_meeting_location);
   explore_interface->goToPoint(current_meeting_location, false);
   return true;
}

bool GoToMeet::isDone()
{
   // Connection established
   
   // Temporary case
   ros::Duration(20).sleep();
   return true;
}


TEAM_STATES GoToMeet::transition() 
{
  if(isDone())
    return MEET;
  else GO_TO_MEET;
}

void GoToMeet::step()
{
  ROS_INFO("Step for GoToMeet");
}

void GoToMeet::exitPoint() 
{
  explore_interface->stopRobot();
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
    return GO_TO_EXPLORE;
  else 
    return MEET;
}

void Meet::step()
{
  ROS_INFO("Step for Meet");
}

void Meet::exitPoint() 
{
  current_meeting_location = next_meeting_location;

  // get time estimate from move_base_interface
  // set exploration_duration accordingly
}
