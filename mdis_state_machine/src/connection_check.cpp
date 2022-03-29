#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <tf/transform_listener.h>

#include <mdis_state_machine/Connection.h>

const int MAX_ATTEMPTS=5;
const float DIST_THRESHOLD=1.414;

geometry_msgs::Point getRobotCurrentPose(const std::string& robot_name, tf::TransformListener& tf_listener)
{
  int attempt = 0;
  bool not_found_tf = true;
  tf::StampedTransform transform;
  geometry_msgs::Point result_transform;

  while (attempt++ < MAX_ATTEMPTS && not_found_tf && ros::ok()){
    try{
      tf_listener.lookupTransform("/"+robot_name+"/map", "/"+robot_name+"/base_footprint",  
                               ros::Time(0), transform);
      
      not_found_tf = false;
    }
    catch (tf::TransformException ex){
      ROS_WARN("%s",ex.what());
      ros::Duration(0.10).sleep();
    }
  }
  
  if(not_found_tf)
    ROS_ERROR("Could not find the transform after %i attempts", MAX_ATTEMPTS);
  else
  {
    result_transform.x = transform.getOrigin().x();
    result_transform.y = transform.getOrigin().y();
  }

  return result_transform;
}

float getDistanceOfPoints(const geometry_msgs::Point &pt_1, const geometry_msgs::Point &pt_2)
{
  float x = pt_1.x-pt_2.x;
  float y = pt_1.y-pt_2.y;
  return std::sqrt(x*x+y*y);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "connection_check");
  ros::NodeHandle nh;
  if (argc != 4 && argc != 2)
  {
    ROS_ERROR("This node must be launched with number of robots as argument");
    return 0;
  }
  int number_of_robots = std::stoi(argv[1]);
  tf::TransformListener tf_listener;
  ros::Publisher conn_pub = nh.advertise<mdis_state_machine::Connection>(nh.getNamespace() + "/connection_check", 1000);

  geometry_msgs::Point data_center_location; 
  data_center_location.x = -6;
  data_center_location.y = -5;
  while(ros::ok())
  {
    mdis_state_machine::Connection conn_msg;
    std::string robot_1 = nh.getNamespace();
    robot_1.erase (0,1); // The namespace has a leading forward slash
                         // Hence deleting the first character
    for (int j=0; j<number_of_robots; j++)
    {
      std::string robot_2 = "tb3_"+std::to_string(j);
      float dist = getDistanceOfPoints(getRobotCurrentPose(robot_1, tf_listener), getRobotCurrentPose(robot_2, tf_listener));
      // ROS_INFO_STREAM("Distance between "<<robot_1<<" and "<<robot_2<<":"<<dist);
      if(dist<DIST_THRESHOLD && robot_1!=robot_2)
      {
        conn_msg.connection_to.data = robot_2;
        conn_msg.distance = dist;
        conn_pub.publish(conn_msg);
      }
    }
    float dist = getDistanceOfPoints(getRobotCurrentPose(robot_1, tf_listener), data_center_location);
    // ROS_INFO_STREAM("Distance between "<<robot_1<<" and "<<data_center<<":"<<dist);
    if(dist<DIST_THRESHOLD)
    {
      conn_msg.connection_to.data = "data_center";
      conn_msg.distance = dist;
      conn_pub.publish(conn_msg);
    }
    ros::Duration(1).sleep();
  }
  return 0;
}