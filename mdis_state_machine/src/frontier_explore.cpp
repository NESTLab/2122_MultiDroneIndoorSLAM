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
std::vector<geometry_msgs::Point> frontiers;
// MoveBaseInterface *explore_interface;


void exploreCB(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == false){
        ROS_INFO("Starting exploration");
        explore = true;
    }
    else{
        ROS_INFO("Pausing exploration");
        explore = false;
        stop_explore = true;
    }
}

void getBestFrontiersCB(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    ROS_INFO(" New Frontier Received");
   for (int i=0;i<msg->poses.size();i++){
     frontiers.push_back(msg->poses.at(i).position);
  }
  frontier_data_received_for_explore = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_explore");
    ros::NodeHandle nh;
    std::string robot_name = nh.getNamespace();
    robot_name.erase (0,1);
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("WAITING FOR MOVE_BASE");
    }

    move_base_msgs::MoveBaseGoal goal;
    // bool explore = false;
    
    ros::Subscriber sub = nh.subscribe(nh.getNamespace() + "/explore/pause_exploration", 1000, exploreCB);
    ros::Subscriber frontier_data_sub_explore = nh.subscribe(nh.getNamespace() + "/frontier_list", 1000, getBestFrontiersCB);
    // ros::Subscriber sub = n.subscribe("tb3_0/explore/pause_exploration", 1000, exploreCB);
    ros::Publisher frontier_req_pub_explore = nh.advertise<std_msgs::String>(nh.getNamespace() + "/frontier_request", 1000);
    
    // ros::Subscriber frontier_data_sub = n.subscribe("tb3_0/frontier_list", 1000, getBestFrontiersCB);
    while (ros::ok())
    {
        if (explore && !stop_explore)
        {
            std_msgs::String data;
            data.data = robot_name;
            ROS_INFO_STREAM("ROBOT NAME "<<robot_name);
            // frontier_req_pub_explore.publish(data);
            while(!frontier_data_received_for_explore)
            {
                ROS_INFO_STREAM("Waiting for frontier data: "<<robot_name);
                frontier_req_pub_explore.publish(data);
                ros::Duration(0.2).sleep();
                ros::spinOnce();
            }
            ROS_INFO_STREAM("This is the frontier explore goal"<<frontiers[0]);
            geometry_msgs::Point temp_point = frontiers[0];
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = temp_point.x;
            goal.target_pose.pose.position.y = temp_point.y;
            goal.target_pose.pose.orientation.w = 1.;
            ac.sendGoal(goal);
            // explore_interface->goToPoint(temp_point, false);
            ros::Duration(5).sleep();
            frontiers.clear();
            frontier_data_received_for_explore = false;
        }
        else if (!explore && stop_explore){
            // explore_interface->stopRobot();
            ros::Duration(5).sleep();
            stop_explore = false;
        }
        ros::spinOnce();
    }
    return 0;
}