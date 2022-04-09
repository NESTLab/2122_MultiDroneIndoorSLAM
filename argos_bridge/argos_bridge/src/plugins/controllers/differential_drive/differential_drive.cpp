#include "differential_drive.h"

using namespace std;

CDifferentialDrive::CDifferentialDrive() : m_pcWheels(NULL),
                                           m_pcEncoder(NULL)
{
}

CDifferentialDrive::Init(TConfigurationNode &t_node)
{
    // Name and ROS
    controller_name = GetId();
    odom_header_frame_id = "odom";
    odom_child_frame_id = "base_footprint";
    CDifferentialDrive::InitROS();
    current_time = ros::Time(0.0);
    previous_time = ros::Time(0.0);

    // Odometry Publisher
    stringstream odomTopic;
    odomTopic << "/" << controller_name << "/odom";
    odom_pub = nh->advertise<nav_msgs::Odometry>(odomTopic.str(), 1);

    // Command Velocity Subsriber
    stringstream cmdVelTopic;
    cmdVelTopic << "/" << controller_name << "/cmd_vel";
    cmd_vel_sub = nh->subscribe(cmdVelTopic.str(), 1, &CDifferentialDrive::CommandVelocityCallback, this);
  
    // Get sensor/actuator handles
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcEncoder   = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");
}

CDifferentialDrive::InitROS()
{
    stringstream name;
    name << controller_name << "_argos_ctrl";
    ros::init(0, "", name.str());
    nh = new ros::NodeHandle();
}

CDifferentialDrive::CommandVelocityCallback(const geometry_msgs::Twist &twist)
{
  Real v = twist.linear.x * 100.0f;  // Forward speed
  Real w = twist.angular.z * 100.0f; // Rotational speed
  if (abs(w) < goStraightConstant){
    w = 0.0;
  }
  leftSpeed = v - BASE_RADIUS * w;
  rightSpeed = v + BASE_RADIUS * w;
  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);

}