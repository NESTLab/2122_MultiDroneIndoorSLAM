#include <move_base_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_test");
  ros::NodeHandle nh;
  MoveBaseInterface explore(nh);
  // ros::Duration(2).sleep();
  geometry_msgs::Point point_1, point_2;
  point_1.x = -31;
  point_1.y = -8;
  point_2.x = -20;
  point_2.y = -5;
  geometry_msgs::PoseStamped pose = explore.getRobotCurrentPose();
  ros::Time start_time = ros::Time::now();

  float dist = explore.getDistancePrediction(pose.pose.position, point_2);
  ROS_INFO_STREAM("Distance: "<<dist);
  explore.goToPoint(point_2, true);
  ros::Time end_time = ros::Time::now();

  ROS_INFO_STREAM("Time Taken: "<<end_time-start_time);

  return 0;
}
