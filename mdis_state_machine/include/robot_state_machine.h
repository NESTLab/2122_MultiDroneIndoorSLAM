#pragma once

#include <move_base_interface.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <mdis_state_machine/Connection.h>
#include <mdis_state_machine/DataCommunication.h>
#include <mdis_state_machine/Location.h>
#include <mdis_state_machine/LocationAck.h>
#include <mdis_state_machine/ConnectionRequest.h>
#include <cmath>
#include <explore_lite/FrontiersArray.h>
#include <std_msgs/String.h>

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
   DUMP_DATA,
   ERROR_STATE,
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
   std::string data_center_name = "data_center";
   static float curr_meet_x, curr_meet_y, next_meet_x, next_meet_y;
   static float time_for_exploration;
   static std::string connected_robot_name;
   static std::string parent_robot_name, child_robot_name;
   static bool testing_mode;
   static ROLE robot_role;
   static TEAM_STATES last_robot_state;

   int testing_waiting_time = 5;

   geometry_msgs::Point data_dump_location;

   bool interested;
   bool is_explorer;
   bool meeting_started, go_for_exploration;

   MoveBaseInterface *explore_interface;
   ros::Publisher robot_state_pub;

   void publishRobotState()
   {
     mdis_state_machine::RobotsState state_pub_data;
     state_pub_data.robot_state = m_unId;
     robot_state_pub.publish(state_pub_data);
   }

   void setCurrentMeetingPoint(const geometry_msgs::Point& meeting)
   {
     curr_meet_x = meeting.x;
     curr_meet_y = meeting.y;
   }
   void setNextMeetingPoint(const geometry_msgs::Point& meeting)//not being used now. This was to set meeting point as the one from which explorer returned
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
      conn_sub = nh.subscribe(nh.getNamespace() + "/connection_check", 1000, &GoToMeet::connCB, this);     
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

   std::string conn_robot;
   ros::Publisher connection_request_pub;
   ros::Subscriber connection_request_sub;
   ros::Subscriber conn_sub;

   void connCB(const mdis_state_machine::Connection::ConstPtr msg);
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
   bool connection_request_received;
   int current_publishing_counter;
   const int LEAST_PUBLISH_COUNT = 5;

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
   bool once;
   bool connection_request_received;
   int current_publishing_counter;
   const int LEAST_PUBLISH_COUNT = 5;
   
   ros::Publisher connection_request_pub;
   ros::Subscriber connection_request_sub;
   
   void connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg);
   void publishConnectionRequest();
};

class GoToDumpData: public RobotState{
public:
   GoToDumpData(ros::NodeHandle &nh, bool testing):RobotState(GO_TO_DUMP_DATA, "GoToDumpData", nh, testing){
      connection_request_sub = nh.subscribe("/connection_request", 1000, &GoToDumpData::connectionRequestCB, this);
      connection_request_pub = nh.advertise<mdis_state_machine::ConnectionRequest>("/connection_request", 1000);     
      conn_sub = nh.subscribe(nh.getNamespace() + "/connection_check", 1000, &GoToDumpData::connCB, this);  
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

   std::string conn_robot;
   ros::Publisher connection_request_pub;
   ros::Subscriber connection_request_sub;
   ros::Subscriber conn_sub;

   void connCB(const mdis_state_machine::Connection::ConstPtr msg);
   void connectionRequestCB(const mdis_state_machine::ConnectionRequest::ConstPtr msg);

   void publishConnectionRequest();
   void sendRobotToLocation();
   std::string getRobotOfInterestName();

   ros::Time time_of_last_conn;
   ros::Duration wait_time_for_conn = ros::Duration(2.0);
};
class DumpData: public RobotState{
public:
   DumpData(ros::NodeHandle &nh, bool testing):RobotState(DUMP_DATA, "DumpData", nh, testing){
   }
   bool isDone() override ;
   TEAM_STATES transition() override;
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
private:
};



