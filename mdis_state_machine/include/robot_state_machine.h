#pragma once

#include <move_base_interface.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <mdis_state_machine/Connection.h>
#include <mdis_state_machine/DataCommunication.h>
#include <mdis_state_machine/Location.h>
#include <mdis_state_machine/LocationAck.h>
#include <mdis_state_machine/Interest.h>
#include <cmath>
#include <explore_lite/FrontiersArray.h>
#include <std_msgs/String.h>

#include <std_msgs/Int8.h>
#include <mdis_state_machine/RobotsState.h>
#include <coms/TriggerMerge.h>

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
   void setParent(const std::string& name){parent_robot_name = name;}
   void setChild(const std::string& name){child_robot_name = name;}
   void setRobotRole(ROLE role){robot_role = role;}
   std::vector<geometry_msgs::Point> robot_locations_during_meeting;
   

protected:

   uint64_t m_unId;
   // ros::Subscriber interest_sub;
   std::string m_strName;
   std::string robot_name;
   static float curr_meet_x, curr_meet_y, next_meet_x, next_meet_y;
   static float time_for_exploration;
   static std::string parent_robot_name, child_robot_name;
   static bool testing_mode;
   static ROLE robot_role;

   int testing_waiting_time = 1;

   geometry_msgs::Point data_dump_location;

   bool interested;
   bool is_explorer;
   bool meeting_started, go_for_exploration;

   MoveBaseInterface *explore_interface;



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
   geometry_msgs::Point getNextFurthestMeetingPoint(const std::vector<geometry_msgs::Point>& frontiers, const std::vector<geometry_msgs::Point>& curr_location_f)
   // geometry_msgs::Point getNextFurthestMeetingPoint(const std::vector<geometry_msgs::Point>& frontiers, const geometry_msgs::Point& curr_location_f)
   {
      float max_dis = 0.0;
      geometry_msgs::Point point;
      geometry_msgs::Point temp_pos;
      // temp_pos.x=curr_location_f.x;
      // temp_pos.y=curr_location_f.y;

      for (auto& frontier : frontiers){//euclidean distance to frontier from explorer meeting point
         geometry_msgs::Point temp;
         // geometry_msgs::Point temp_pos;
         temp.x=frontier.x;
         temp.y=frontier.y;
         float total_distance_from_frontier = 0;
         for (int i=0;i<curr_location_f.size();i++){
            temp_pos.x=curr_location_f[i].x;
            temp_pos.y=curr_location_f[i].y;
            float distance_from_frontier = explore_interface->getDistancePrediction(temp, temp_pos);
            total_distance_from_frontier += distance_from_frontier;
         }
         // float total_distance_from_frontier = explore_interface->getDistancePrediction(temp, temp_pos);
         if (total_distance_from_frontier>max_dis){
            max_dis = total_distance_from_frontier;
            point.x=frontier.x;
            point.y=frontier.y;
            setNextMeetingPoint(point);
            
         }

      }
     return point;
   }
   geometry_msgs::Point getNextMeetingPoint()
   {
     geometry_msgs::Point point;
     point.x = next_meet_x;
     point.y = next_meet_y;
     return point;
   }
   geometry_msgs::Point getNextClosestMeetingPoint(const std::vector<geometry_msgs::Point>& frontiers, const geometry_msgs::Point& curr_location_f)
   {
      float min_dis = LONG_MAX;
      geometry_msgs::Point point;
      for (auto& frontier : frontiers){
         geometry_msgs::Point temp;
         temp.x=frontier.x;
         temp.y=frontier.y;
         float distance_from_frontier = explore_interface->getDistancePrediction(temp);
         // float distance_from_frontier = sqrt(((curr_location_f.x-frontier.x)*(curr_location_f.x-frontier.x))+((curr_location_f.y-frontier.y)*(curr_location_f.y-frontier.y)));
         if (distance_from_frontier<min_dis){
            min_dis = distance_from_frontier;
            point.x=frontier.x;
            point.y=frontier.y;
         }
      }
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
     interest_sub = nh.subscribe("/interest_check", 1000, &GoToMeet::interestCB, this);
     interest_pub = nh.advertise<mdis_state_machine::Interest>("/interest_check", 1000);     
   }
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool connected;
   std::string conn_robot;
   ros::Publisher interest_pub;
   ros::Subscriber interest_sub;
   ros::Publisher robot_state_pub;
   ros::Subscriber conn_sub;
   mdis_state_machine::RobotsState state_pub_data;
   void connCB(const mdis_state_machine::Connection::ConstPtr msg);
   void interestCB(const mdis_state_machine::Interest::ConstPtr msg);
};
class Meet: public RobotState{
public:
   Meet(ros::NodeHandle &nh, bool testing):RobotState(MEET, "Meet", nh, testing){
     meeting_data_pub = nh.advertise<mdis_state_machine::DataCommunication>("/data_communication", 1000);
     meeting_data_sub = nh.subscribe("/data_communication", 1000, &Meet::nextMeetingLocationCB, this);
   //   frontier_data_sub = nh.subscribe("/frontier_list", 1000, &Meet::getFrontiersCB, this);
     frontier_data_sub = nh.subscribe("/tb3_0/frontier_list", 1000, &Meet::getBestFrontiersCB, this);
     location_data_pub = nh.advertise<mdis_state_machine::Location>("/robot_location", 1000);
     location_data_sub = nh.subscribe("/robot_location", 1000, &Meet::getLocationCB, this);
     location_data_ack_pub = nh.advertise<mdis_state_machine::LocationAck>("/robot_location_ack", 1000);
     location_data_ack_sub = nh.subscribe("/robot_location_ack", 1000, &Meet::getLocationAckCB, this);
     robot_state_pub = nh.advertise<mdis_state_machine::RobotsState>("/robots_state", 1000);
     frontier_req_pub = nh.advertise<std_msgs::String>("/frontier_request", 1000);   

     mergeRequestClient = nh.serviceClient<coms::TriggerMerge>(srv_name);
   }

  std::string srv_name = "trigger_merge";
   bool isDone() override ;
   TEAM_STATES transition() override;
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   ros::Publisher meeting_data_pub;
   ros::Publisher frontier_req_pub;
   ros::Subscriber meeting_data_sub;
   ros::Subscriber frontier_data_sub;
   // frontier_exploration::FrontierSearch search_;
   ros::Publisher location_data_pub;
   ros::Subscriber location_data_sub;
   ros::Publisher location_data_ack_pub;
   ros::Subscriber location_data_ack_sub;
   ros::Publisher robot_state_pub;
   ros::ServiceClient mergeRequestClient;
   mdis_state_machine::RobotsState state_pub_data;


   bool data_received;
   bool frontier_data_received;
   bool location_data_received;
   bool location_data_ack;
   void getFrontiersCB(const explore_lite::FrontiersArray::ConstPtr msg);
   void getBestFrontiersCB(const geometry_msgs::PoseArray::ConstPtr msg);
   void requestMerge(std::string conn_robot);
   void publishNextMeetingLocation();
   void nextMeetingLocationCB(const mdis_state_machine::DataCommunication::ConstPtr msg);
   void getLocationCB(const mdis_state_machine::Location::ConstPtr msg);
   void getLocationAckCB(const mdis_state_machine::LocationAck::ConstPtr msg);
   void getNextMeetingLocationFromCallback();
   void setExplorationTime();
   void getLocations();
   geometry_msgs::Point buffer_next_location;
   std::vector<geometry_msgs::Point> frontier_msg;
   geometry_msgs::Point location_msg;
   bool is_merge_complete;

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



