#pragma once

#include <move_base_interface.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <mdis_state_machine/RobotsState.h>
#include <mdis_state_machine/Connection.h>
#include <mdis_state_machine/DataCommunication.h>
enum ROLE{
  RELAY,
  EXPLORER,
  RELAY_BETN_ROBOTS,
};

enum TEAM_STATES{
   IDLE,
   GO_TO_EXPLORE,
   EXPLORE,
   GO_TO_MEET,
   MEET,
   GO_TO_DUMP_DATA,
   DUMP_DATA,
};

class RobotState {
   
public:

   RobotState(uint64_t un_id, const std::string& str_name, ros::NodeHandle &nh, bool testing);

   ~RobotState() 
   {
   }

   uint64_t getId() const { return m_unId; }

   const std::string& getName() const { return m_strName; }
   
   virtual bool entryPoint() = 0;
   
   virtual void exitPoint() = 0;
   
   virtual void step() = 0;
   
   virtual bool isDone() = 0;

   virtual TEAM_STATES transition() = 0;

   void setParent(const std::string& name)
   {
     parent_robot_name = name;
   }

   void setChild(const std::string& name)
   {
     child_robot_name = name;
   }
   
   void setRobotRole(ROLE role)
   {
     robot_role = role;
   }
protected:

   uint64_t m_unId;
   std::string m_strName;
   std::string robot_name;
   static float curr_meet_x, curr_meet_y, next_meet_x, next_meet_y;
   static float time_for_exploration;
   static std::string parent_robot_name, child_robot_name;
   static bool testing_mode;
   static ROLE robot_role;

   int testing_waiting_time = 5;

   geometry_msgs::Point data_dump_location;

   bool is_explorer;
   bool meeting_started, go_for_exploration;

   MoveBaseInterface *explore_interface;



   void setCurrentMeetingPoint(const geometry_msgs::Point& meeting)
   {
     curr_meet_x = meeting.x;
     curr_meet_y = meeting.y;
   }
   void setNextMeetingPoint(const geometry_msgs::Point& meeting)
   {
     next_meet_x = meeting.x;
     next_meet_y = meeting.y;
   }
   geometry_msgs::Point getCurrentMeetingPoint()
   {
     geometry_msgs::Point point;
     point.x = curr_meet_x;
     point.y = curr_meet_y;
     return point;
   }
   geometry_msgs::Point getNextMeetingPoint()
   {
     geometry_msgs::Point point;
     point.x = next_meet_x;
     point.y = next_meet_y;
     return point;
   }
   void setCurrAsNextMeeting()
   {
     curr_meet_x = next_meet_x;
     curr_meet_y = next_meet_y;
   }
   inline bool isConnDirectRelated(const std::string& conn_robot)
   {
     bool conn_parent = (conn_robot == parent_robot_name) && (conn_robot != "");
     bool conn_child = (conn_robot == child_robot_name) && (conn_robot != "");
     return ((conn_parent) || (conn_child));
   }
   
};

class Idle: public RobotState{
public:
   Idle(ros::NodeHandle &nh, bool testing):RobotState(IDLE, "Idle", nh, testing){}
   bool isDone() override ;

   TEAM_STATES transition() override;
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   ros::Publisher chatter_pub;
};


class GoToExplore: public RobotState{
public:
   GoToExplore(ros::NodeHandle &nh, bool testing):RobotState(GO_TO_EXPLORE, "GoToExplore", nh, testing){
   robot_state_pub = nh.advertise<mdis_state_machine::RobotsState>("/robots_state", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   ros::Publisher robot_state_pub;
   mdis_state_machine::RobotsState state_pub_data;
};


class Explore: public RobotState{
public:
   Explore(ros::NodeHandle &nh, bool testing):RobotState(EXPLORE, "Explore", nh, testing){
     pause_exploration_pub = nh.advertise<std_msgs::Bool>("explore/pause_exploration", 1000);     
     robot_state_pub = nh.advertise<mdis_state_machine::RobotsState>("/robots_state", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   ros::Publisher pause_exploration_pub;
   ros::Publisher robot_state_pub;
   mdis_state_machine::RobotsState state_pub_data;
   ros::Time starting_time;
};


class GoToMeet: public RobotState{
public:
   GoToMeet(ros::NodeHandle &nh, bool testing):RobotState(GO_TO_MEET, "GoToMeet", nh, testing){
     conn_sub = nh.subscribe("/connection_check", 1000, &GoToMeet::connCB, this);     
   robot_state_pub = nh.advertise<mdis_state_machine::RobotsState>("/robots_state", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool connected;
   std::string conn_robot;
   ros::Subscriber conn_sub;
   ros::Publisher robot_state_pub;
   mdis_state_machine::RobotsState state_pub_data;

   void connCB(const mdis_state_machine::Connection::ConstPtr msg);
};


class Meet: public RobotState{
public:
   Meet(ros::NodeHandle &nh, bool testing):RobotState(MEET, "Meet", nh, testing){
     meeting_data_pub = nh.advertise<mdis_state_machine::DataCommunication>("/data_communication", 1000);
     meeting_data_sub = nh.subscribe("/data_communication", 1000, &Meet::nextMeetingLocationCB, this);     
     robot_state_pub = nh.advertise<mdis_state_machine::RobotsState>("/robots_state", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   ros::Publisher meeting_data_pub;
   ros::Publisher robot_state_pub;
   mdis_state_machine::RobotsState state_pub_data;
   ros::Subscriber meeting_data_sub;

   bool data_received;

   void publishNextMeetingLocation();
   void nextMeetingLocationCB(const mdis_state_machine::DataCommunication::ConstPtr msg);
   void getNextMeetingLocationFromCallback();
   void setExplorationTime();
   geometry_msgs::Point buffer_next_location;
};


class GoToDumpData: public RobotState{
public:
   GoToDumpData(ros::NodeHandle &nh, bool testing):RobotState(GO_TO_DUMP_DATA, "GoToDumpData", nh, testing){
   robot_state_pub = nh.advertise<mdis_state_machine::RobotsState>("/robots_state", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   ros::Publisher robot_state_pub;
   mdis_state_machine::RobotsState state_pub_data;
};


class DumpData: public RobotState{
public:
   DumpData(ros::NodeHandle &nh, bool testing):RobotState(DUMP_DATA, "DumpData", nh, testing){
   robot_state_pub = nh.advertise<mdis_state_machine::RobotsState>("/robots_state", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   ros::Publisher robot_state_pub;
   mdis_state_machine::RobotsState state_pub_data;
};



