#pragma once

#include <move_base_interface.h>
#include <std_msgs/Bool.h>

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

   RobotState(uint64_t un_id, const std::string& str_name, ros::NodeHandle &nh);

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

   geometry_msgs::Point current_meeting_location, next_meeting_location;
   
protected:

   uint64_t m_unId;
   std::string m_strName;

   bool is_explorer;
   bool meeting_started, go_for_exploration;

   MoveBaseInterface *explore_interface;

   ros::Duration time_until_next_meeting = ros::Duration(30.0);
};

class Idle: public RobotState{
public:
   Idle(ros::NodeHandle &nh):RobotState(IDLE, "Idle", nh){}
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
   GoToExplore(ros::NodeHandle &nh):RobotState(GO_TO_EXPLORE, "GoToExplore", nh){}
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
};


class Explore: public RobotState{
public:
   Explore(ros::NodeHandle &nh):RobotState(EXPLORE, "Explore", nh){
     pause_exploration_pub = nh.advertise<std_msgs::Bool>("explore/pause_exploration", 1000);
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
   GoToMeet(ros::NodeHandle &nh):RobotState(GO_TO_MEET, "GoToMeet", nh){}
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
};


class Meet: public RobotState{
public:
   Meet(ros::NodeHandle &nh):RobotState(MEET, "Meet", nh){}
   bool isDone() override ;

   TEAM_STATES transition() override;
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
};



