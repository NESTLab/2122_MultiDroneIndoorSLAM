#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseArray.h"
#include <vector>
#include <move_base_interface.h>
#include "geometry_msgs/Point.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


bool explore = false;
bool stop_explore = false;
bool frontier_data_received_for_explore = false;
geometry_msgs::Point frontier;
MoveBaseInterface *explore_interface;
// MoveBaseInterface *explore_interface;


void pauseExploreCB(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data){
        ROS_INFO("Pausing exploration");
        explore = false;
        stop_explore = true;
    }
    else{
        ROS_INFO("Starting exploration");
        explore = true;
    }
}

void getBestFrontiersCB(const geometry_msgs::PoseArray& msg)
{
    ROS_INFO(" New Frontier Received");
   if (msg.poses.size()==0)
   {
      ROS_WARN("Empty frontier list encountered");
      return;
   }

  float min_dis=100000.0;
  for (auto& curr_frontier : msg.poses)
  {
    //euclidean distance to frontier from explorer meeting point
    float self_distance_from_frontier = explore_interface->getDistancePrediction(curr_frontier.position); //experimenting new method
  
    if (self_distance_from_frontier<=min_dis){
      min_dis = self_distance_from_frontier;
      frontier=curr_frontier.position;      
    }
  }
   frontier = msg.poses.at(0).position;
   
  frontier_data_received_for_explore = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_explore");
    ros::NodeHandle nh;
    std::string robot_name = nh.getNamespace();
    robot_name.erase (0,1);
    
    explore_interface = new MoveBaseInterface(nh, false);
    
    ros::Subscriber sub = nh.subscribe(nh.getNamespace() + "/explore/pause_exploration", 1000, pauseExploreCB);
    ros::Subscriber frontier_data_sub_explore = nh.subscribe(nh.getNamespace() + "/frontier_list", 1000, getBestFrontiersCB);
    ros::Publisher frontier_req_pub_explore = nh.advertise<std_msgs::String>(nh.getNamespace() + "/frontier_request", 1000);
    
    while (ros::ok())
    {
        if (explore && !stop_explore)
        {
            std_msgs::String data;
            data.data = robot_name;
            ROS_INFO_STREAM("ROBOT NAME "<<robot_name);
            // frontier_req_pub_explore.publish(data);
            bool request=true;
            while(!frontier_data_received_for_explore && ros::ok())
            {
                ROS_INFO_STREAM("Waiting for frontier data: "<<robot_name);
                if(request)
                  frontier_req_pub_explore.publish(data);
                request=false;
                ros::Duration(0.1).sleep();
                ros::spinOnce();
            }
            explore_interface->goToPoint(frontier, false);
            frontier_data_received_for_explore = false;
            ros::Duration(05).sleep();
        }
        else if (!explore && stop_explore){
            if(explore)
                explore_interface->stopRobot();
            ros::Duration(0.5).sleep();
            stop_explore = false;
        }
        ros::spinOnce();
    }
    return 0;
}