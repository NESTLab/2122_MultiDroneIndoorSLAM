#include <move_base_interface.h>

MoveBaseInterface::MoveBaseInterface(ros::NodeHandle nh): nh_(nh)
{
  move_base_client_ = new MoveBaseTypedef("move_base", true);
  move_base_planning_client_ = nh.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
  debug_generated_path_pub = nh.advertise<nav_msgs::Path>("debug_plan_path", 1000);

  ROS_INFO_STREAM(logging_prefix_ << "Waiting for actionlib servers");
  move_base_client_->waitForServer();
  ROS_INFO_STREAM(logging_prefix_ << "All servers started");

  namespace_prefix.erase(namespace_prefix.begin());
  map_frame = namespace_prefix+"/map";
  robot_frame = namespace_prefix+"/base_footprint";
}

bool MoveBaseInterface::goToPose(geometry_msgs::PoseStamped &goal_pose, bool wait_for_result)
{
  move_base_action_goal_.target_pose = goal_pose;
  move_base_client_->sendGoal(move_base_action_goal_);
  ROS_INFO_STREAM(logging_prefix_ << "Waiting");
  if(wait_for_result)
  {
    move_base_client_->waitForResult();
    
    // Doesn't work, 
    // @to-do fix later
    // return move_base_client_->getState().status_list.front() == actionlib_msgs::GoalStatus::SUCCEEDED;
  }
  return true;
}

bool MoveBaseInterface::goToPoint(geometry_msgs::Point &goal, bool wait_for_result)
{
  move_base_action_goal_.target_pose.pose.position = goal;
  move_base_action_goal_.target_pose.pose.orientation.w = 1.;
  move_base_action_goal_.target_pose.header.frame_id = namespace_prefix+"/map";
  move_base_action_goal_.target_pose.header.stamp = ros::Time::now();
  move_base_client_->sendGoal(move_base_action_goal_);
  ROS_INFO_STREAM(logging_prefix_ << "Waiting");
  if(wait_for_result)
  {
    move_base_client_->waitForResult();
    
    // Doesn't work, 
    // @to-do fix later
    // return move_base_client_->getState().status_list.front() == actionlib_msgs::GoalStatus::SUCCEEDED;
  }
  return true;
}

bool MoveBaseInterface::stopRobot()
{
  move_base_client_->cancelGoal();
  return true;
}

float MoveBaseInterface::getDistancePrediction(geometry_msgs::PoseStamped &start_pose, geometry_msgs::PoseStamped &end_pose)
{
  nav_msgs::GetPlan srv;
  srv.request.start = start_pose;
  srv.request.goal = end_pose;
  move_base_planning_client_.call(srv);
  nav_msgs::Path path = srv.response.plan;
  return calculatePathLength(path);
}

float MoveBaseInterface::getDistancePrediction(geometry_msgs::Point &start_point, geometry_msgs::Point &end_point)
{
  nav_msgs::GetPlan srv;
  srv.request.start.pose.position = start_point;
  srv.request.start.pose.orientation.w = 1.;
  srv.request.start.header.frame_id = map_frame;
  srv.request.start.header.stamp = ros::Time::now();

  srv.request.goal.pose.position = end_point;
  srv.request.goal.pose.orientation.w = 1.;
  srv.request.goal.header.frame_id = map_frame;
  srv.request.goal.header.stamp = ros::Time::now();
  
  move_base_planning_client_.call(srv);
  nav_msgs::Path path = srv.response.plan;
  path.header.frame_id = map_frame;
  path.header.stamp = ros::Time::now();
  debug_generated_path_pub.publish(path);
  return calculatePathLength(path);
}

float MoveBaseInterface::getDistancePrediction(geometry_msgs::Point &goal)
{
  nav_msgs::GetPlan srv;
  srv.request.start = getRobotCurrentPose();

  srv.request.goal.pose.position = goal;
  srv.request.goal.pose.orientation.w = 1.;
  srv.request.goal.header.frame_id = map_frame;
  srv.request.goal.header.stamp = ros::Time::now();
  
  move_base_planning_client_.call(srv);
  nav_msgs::Path path = srv.response.plan;
  path.header.frame_id = map_frame;
  path.header.stamp = ros::Time::now();
  debug_generated_path_pub.publish(path);
  return calculatePathLength(path);
}

float MoveBaseInterface::calculatePathLength(const nav_msgs::Path& path)
{
  double length = 0;
  float last_x, last_y;
  for(int i = 0; i<path.poses.size(); i++)
  {
    float x = path.poses.at(i).pose.position.x;
    float y = path.poses.at(i).pose.position.y;
    if(i != 0)
      length += std::sqrt((x-last_x)*(x-last_x)+(y-last_y)*(y-last_y));\
    last_x = x;
    last_y = y;
  }
  return length;
}

geometry_msgs::PoseStamped MoveBaseInterface::getRobotCurrentPose()
{
  int attempt = 0;
  bool not_found_tf = true;
  tf::StampedTransform transform;
  geometry_msgs::PoseStamped result_transform;

  result_transform.header.frame_id = map_frame;
  result_transform.header.stamp = ros::Time::now();

  while (attempt++ < MAX_ATTEMPTS && not_found_tf && ros::ok()){
    try{
      tf_listener_.lookupTransform(map_frame, robot_frame,  
                               ros::Time(0), transform);
      
      not_found_tf = false;
    }
    catch (tf::TransformException ex){
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  
  if(not_found_tf)
    ROS_INFO("Could not find the transform after %i attempts", MAX_ATTEMPTS);
  else
    result_transform = transformTf2msg(transform);

  return result_transform;
}

geometry_msgs::PoseStamped MoveBaseInterface::transformTf2msg(const tf::StampedTransform& transform)
{
  geometry_msgs::PoseStamped msg;
  
  msg.pose.position.x = transform.getOrigin().x();
  msg.pose.position.y = transform.getOrigin().y();
  msg.pose.position.z = transform.getOrigin().z();
  
  msg.pose.orientation.x = transform.getRotation().x();
  msg.pose.orientation.y = transform.getRotation().y();
  msg.pose.orientation.z = transform.getRotation().z();
  msg.pose.orientation.w = transform.getRotation().w();

  return msg;
}

float MoveBaseInterface::getTimePredictionForTravel(geometry_msgs::Point &goal)
{
  float distance = getDistancePrediction(goal);
  float time = distance/ROBOT_SPEED;
  return time;
}

float MoveBaseInterface::getTimePredictionForTravel(geometry_msgs::Point &start_point, geometry_msgs::Point &end_point)
{
  float distance = getDistancePrediction(start_point, end_point);
  float time = distance/ROBOT_SPEED;
  return time;
}

bool MoveBaseInterface::navigationSucceeded()
{
  bool success = move_base_client_->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
  return success;
}

bool MoveBaseInterface::navigationDone()
{
  bool done = move_base_client_->getState().isDone();
  return done;
}