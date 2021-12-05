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
  point_2.x = -31;
  point_2.y = -6;
  float dist = explore.getDistancePrediction(point_2);
  ROS_INFO_STREAM("Distance: "<<dist);

  geometry_msgs::PoseStamped pose = explore.getRobotCurrentPose();

  ROS_INFO_STREAM("Current_pose: "<<pose.pose);

  return 0;
}
