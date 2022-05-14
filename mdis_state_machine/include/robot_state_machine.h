#pragma once

#include <move_base_interface.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <mdis_state_machine/Connection.h>
#include <mdis_state_machine/ConnectionRequest.h>
#include <cmath>
#include <explore_lite/FrontiersArray.h>
#include <std_msgs/String.h>
#include "argos_bridge/losList.h"

#include <std_msgs/Int8.h>
#include <mdis_state_machine/RobotsState.h>
#include <coms/TriggerMerge.h>
#include <time.h>

enum ROLE{
  RELAY,
  EXPLORER,
  DATA_CENTER,
};

enum TEAM_STATES{
   IDLE,
   GO_TO_EXPLORE,
   EXPLORE,
   GO_TO_MEET,
   TRANSIT_TO_MEET,
   MERGE_MAP,
   DECIDE_NEXT_MEETING,
   RECEIVE_NEXT_MEETING,
   END_MEETING,
   GO_TO_DUMP_DATA,
   DATA_CENTER_READY_TO_MEET,
   ERROR_STATE=99,
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
   void setParent(const std::string& name){parent_robot_name = name;}
   void setChild(const std::string& name){child_robot_name = name;}
   void setRobotRole(ROLE role){robot_role = role;}
   std::vector<geometry_msgs::Point> robot_locations_during_meeting;
   

protected:

   uint64_t m_unId;
   // ros::Subscriber interest_sub;
   std::string m_strName;
   std::string robot_name;
   std::string data_center_name = "Khepera_3";
   static float explore_loc_x, explore_loc_y;
   static float meet_loc_x, meet_loc_y;
   
   static float time_for_exploration;
   static std::string connected_robot_name;
   static std::string parent_robot_name, child_robot_name;
   static bool testing_mode;
   static ROLE robot_role;
   static TEAM_STATES last_robot_state;

   int testing_waiting_time = 5;
   int robot_name_length = 9;

   geometry_msgs::Point data_dump_location;

   bool interested;
   bool is_explorer;
   bool meeting_started, go_for_exploration;
   const float MIN_TIME_FOR_EXPLORATION = 180;

   MoveBaseInterface *explore_interface;
   ros::Publisher robot_state_pub;

   void publishRobotState()
   {
     mdis_state_machine::RobotsState state_pub_data;
     state_pub_data.robot_state = m_unId;
     robot_state_pub.publish(state_pub_data);
   }

   void setGoToExplorePoint(const geometry_msgs::Point& explore_point)
   {
      std::string message = getFormattedMessage("Exploration point setting to: ");
      ROS_ERROR_STREAM(message<<explore_point);
     explore_loc_x = explore_point.x;
     explore_loc_y = explore_point.y;
   }

   void setMeetingPoint(const geometry_msgs::Point& meeting)
   {
      std::string message = getFormattedMessage("Meeting point setting to: ");
      ROS_ERROR_STREAM(message<<meeting);
     meet_loc_x = meeting.x;
     meet_loc_y = meeting.y;
   }

   geometry_msgs::Point getExplorePoint()
   {
     geometry_msgs::Point point;
     point.x = explore_loc_x;
     point.y = explore_loc_y;
     return point;
   }

   geometry_msgs::Point getMeetingPoint()
   {
     geometry_msgs::Point point;
     point.x = meet_loc_x;
     point.y = meet_loc_y;
     return point;
   }



   inline bool isConnDirectRelated(const std::string& conn_robot)
   {
     bool conn_parent = (conn_robot == parent_robot_name) && (conn_robot != "");
     bool conn_child = (conn_robot == child_robot_name) && (conn_robot != "");
     return ((conn_parent) || (conn_child));
   }
   
   inline const std::string getFormattedMessage(const std::string& msg)
   {
     return "[ "+robot_name+" | mdis_state_machine | robot_state_machine]: " + msg;
   }

   inline void printMessage(const std::string& msg)
   {
     std::string full_msg = getFormattedMessage(msg);
     ROS_INFO("%s", &full_msg[0]);
   }

   inline void printMessageThrottled(const std::string& msg)
   {
     std::string full_msg = getFormattedMessage(msg);
     ROS_INFO_THROTTLE(1, "%s", &full_msg[0]);
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

class ErrorState: public RobotState{
public:
   ErrorState(ros::NodeHandle &nh, bool testing):RobotState(ERROR_STATE, "ErrorState", nh, testing){
   robot_cmd_vel = nh.advertise<geometry_msgs::Twist>(nh.getNamespace() + "/cmd_vel", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   ros::Publisher robot_cmd_vel;
   geometry_msgs::Twist twist_msg;
   int direction = 1;
};


class GoToExplore: public RobotState{
public:
   GoToExplore(ros::NodeHandle &nh, bool testing):RobotState(GO_TO_EXPLORE, "GoToExplore", nh, testing){
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool send_once;
};


class Explore: public RobotState{
public:
   Explore(ros::NodeHandle &nh, bool testing):RobotState(EXPLORE, "Explore", nh, testing){
     pause_exploration_pub = nh.advertise<std_msgs::Bool>(nh.getNamespace() + "/explore/pause_exploration", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   ros::Publisher pause_exploration_pub;
   ros::Time starting_time;
};


class GoToMeet: public RobotState{
public:
   GoToMeet(ros::NodeHandle &nh, bool testing):RobotState(GO_TO_MEET, "GoToMeet", nh, testing){
      conn_sub = nh.subscribe(nh.getNamespace() + "/lineOfSight", 1000, &GoToMeet::connCB, this);     
      connection_request_sub = nh.subscribe("/connection_request", 1000, &GoToMeet::connectionRequestCB, this);
      connection_request_pub = nh.advertise<mdis_state_machine::ConnectionRequest>("/connection_request", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool connected;
   bool connection_request_received;
   bool send_once;
   bool this_state = false;
   bool robot_moving;

   std::string conn_robot;
   ros::Publisher connection_request_pub;
   ros::Subscriber connection_request_sub;
   ros::Subscriber conn_sub;

   void connCB(const argos_bridge::losList::ConstPtr msg);
   void connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg);

   void publishConnectionRequest();
   void sendRobotToLocation();
   std::string getRobotOfInterestName();

   ros::Time time_of_last_conn;
   ros::Duration wait_time_for_conn = ros::Duration(2.0);
};

class TransitToMeet: public RobotState{
public:
   TransitToMeet(ros::NodeHandle &nh, bool testing):RobotState(TRANSIT_TO_MEET, "TransitToMeet", nh, testing){      
    connection_request_sub = nh.subscribe("/connection_request", 1000, &TransitToMeet::connectionRequestCB, this);
    connection_request_pub = nh.advertise<mdis_state_machine::ConnectionRequest>("/connection_request", 1000);     
}
   bool isDone() override ;

   TEAM_STATES transition() override;
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool this_state = false;
   bool connection_request_received;
   int current_publishing_counter;
   const int LEAST_PUBLISH_COUNT = 5;
   const int MAX_PUBLISH_COUNT = 50;

   ros::Publisher connection_request_pub;
   ros::Subscriber connection_request_sub;
   void connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg);
   void publishConnectionRequest();

};

class MergeMap: public RobotState{
public:
   MergeMap(ros::NodeHandle &nh, bool testing):RobotState(MERGE_MAP, "MergeMap", nh, testing){
     mergeRequestClient = nh.serviceClient<coms::TriggerMerge>(srv_name);
   }

   bool isDone() override ;
   TEAM_STATES transition() override;
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   std::string srv_name = "trigger_merge";
   bool is_merge_complete;

   ros::ServiceClient mergeRequestClient;
   void requestMerge(std::string conn_robot);
};

class DecideNextMeeting: public RobotState{
public:
   DecideNextMeeting(ros::NodeHandle &nh, bool testing):RobotState(DECIDE_NEXT_MEETING, "DecideNextMeeting", nh, testing){
      frontier_req_pub = nh.advertise<std_msgs::String>(nh.getNamespace() + "/frontier_request", 1000);   
      frontier_data_sub = nh.subscribe(nh.getNamespace() + "/frontier_list", 1000, &DecideNextMeeting::getBestFrontiersCB, this);
   }

   bool isDone() override ;
   TEAM_STATES transition() override;
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool updated_meeting_location;
   bool this_state = false;
   bool frontier_received;

   ros::Publisher frontier_req_pub;
   ros::Subscriber frontier_data_sub;
   std::vector<geometry_msgs::Point> frontiers_list;

   void getBestFrontiersCB(const geometry_msgs::PoseArray::ConstPtr msg);
   void requestFrontiers();
   void updateNextMeetingPoint();
};

class ReceiveNextMeeting: public RobotState{
public:
   ReceiveNextMeeting(ros::NodeHandle &nh, bool testing):RobotState(RECEIVE_NEXT_MEETING, "ReceiveNextMeeting", nh, testing){
    connection_request_sub = nh.subscribe("/connection_request", 1000, &ReceiveNextMeeting::connectionRequestCB, this);
   }

   bool isDone() override ;
   TEAM_STATES transition() override;
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool this_state = false;
   bool connection_request_received;
   ros::Subscriber connection_request_sub;
   void connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg);
};

class EndMeeting: public RobotState{
public:
   EndMeeting(ros::NodeHandle &nh, bool testing):RobotState(END_MEETING, "EndMeeting", nh, testing){
    connection_request_sub = nh.subscribe("/connection_request", 1000, &EndMeeting::connectionRequestCB, this);
    connection_request_pub = nh.advertise<mdis_state_machine::ConnectionRequest>("/connection_request", 1000);     
   }

   bool isDone() override ;
   TEAM_STATES transition() override;
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool this_state = false;
   float meeting_in_time;
   bool once;
   bool connection_request_received;
   int current_publishing_counter;
   const int LEAST_PUBLISH_COUNT = 5;
   
   ros::Publisher connection_request_pub;
   ros::Subscriber connection_request_sub;
   
   void connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg);
   void publishConnectionRequest();
   void calculateTimeForMeeting();
   void calculateTimeForExploration();
};

class GoToDumpData: public RobotState{
public:
   GoToDumpData(ros::NodeHandle &nh, bool testing):RobotState(GO_TO_DUMP_DATA, "GoToDumpData", nh, testing){
      connection_request_sub = nh.subscribe("/connection_request", 1000, &GoToDumpData::connectionRequestCB, this);
      connection_request_pub = nh.advertise<mdis_state_machine::ConnectionRequest>("/connection_request", 1000);     
      conn_sub = nh.subscribe(nh.getNamespace() + "/lineOfSight", 1000, &GoToDumpData::connCB, this);  
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool this_state = false;
   bool connected;
   bool connection_request_received;
   bool send_once;

   std::string conn_robot;
   ros::Publisher connection_request_pub;
   ros::Subscriber connection_request_sub;
   ros::Subscriber conn_sub;

   void connCB(const argos_bridge::losList::ConstPtr msg);
   void connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg);

   void publishConnectionRequest();
   void sendRobotToLocation();
   std::string getRobotOfInterestName();

   ros::Time time_of_last_conn;
   ros::Duration wait_time_for_conn = ros::Duration(2.0);
};

class DataCenterReadyToMeet: public RobotState{
public:
   DataCenterReadyToMeet(ros::NodeHandle &nh, bool testing):RobotState(DATA_CENTER_READY_TO_MEET, "DataCenterReadyToMeet", nh, testing){
      connection_request_sub = nh.subscribe("/connection_request", 1000, &DataCenterReadyToMeet::connectionRequestCB, this);
      connection_request_pub = nh.advertise<mdis_state_machine::ConnectionRequest>("/connection_request", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool this_state = false;
   bool connection_request_received;
   bool rotate_once = true;

   int downtime_counter;
   const int MAX_DOWNTIME = 10;

   std::string conn_robot;
   ros::Publisher connection_request_pub;
   ros::Subscriber connection_request_sub;

   void connCB(const argos_bridge::losList::ConstPtr msg);
   void connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg);

   void publishConnectionRequest();
   void sendRobotToLocation();
   std::string getRobotOfInterestName();

   ros::Time time_of_last_conn;
   ros::Duration wait_time_for_conn = ros::Duration(2.0);
};
