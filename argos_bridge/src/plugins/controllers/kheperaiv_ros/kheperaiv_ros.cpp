/* Include ros */
#include "ros/ros.h"
#include "argos_bridge/Proximity.h"
#include "argos_bridge/ProximityList.h"
#include "argos_bridge/los.h"
#include "argos_bridge/losList.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"

/* Include the controller definition */
#include "kheperaiv_ros.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

#include <iostream>
#include <memory>
#include <sstream>

#include <tf/transform_broadcaster.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

using namespace std;
using namespace argos_bridge;

// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CKheperaIVRos class for each ARGoS robot.
ros::NodeHandle *initROS()
{
  int argc = 0;
  char *argv = (char *)"";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle *CKheperaIVRos::nodeHandle = initROS();

/****************************************/
/****************************************/

CKheperaIVRos::CKheperaIVRos() : m_pcWheels(NULL),
                                 m_pcProximity(NULL),
                                 m_fWheelVelocity(2.5f),
                                 m_pcRABA(NULL),
                                 m_pcRABS(NULL),
                                 m_pcLIDAR(NULL)
{
  odom_broadcaster = std::make_unique<tf::TransformBroadcaster>();
}
/****************************************/
/****************************************/

void CKheperaIVRos::Init(TConfigurationNode &t_node)
{
  // Create the topics to publish
  stringstream proximityTopic;
  proximityTopic << "/" << GetId() << "/proximity";
  proximityPub = nodeHandle->advertise<ProximityList>(proximityTopic.str(), 1);
  stringstream losTopic;
  losTopic << "/" << GetId() << "/lineOfSight";
  losPub = nodeHandle->advertise<losList>(losTopic.str(), 1);
  stringstream odomTopic;
  odomTopic << "/" << GetId() << "/odom";
  odomPub = nodeHandle->advertise<nav_msgs::Odometry>(odomTopic.str(), 1);
  stringstream laserScanTopic;
  laserScanTopic << "/" << GetId()  << "/scan";
  laserScanPub = nodeHandle->advertise<sensor_msgs::LaserScan>(laserScanTopic.str(), 1);

  // Create the subscribers
  stringstream cmdVelTopic;
  cmdVelTopic << "/" << GetId() << "/cmd_vel";
  cmdVelSub = nodeHandle->subscribe(cmdVelTopic.str(), 1, &CKheperaIVRos::cmdVelCallback, this);

  stringstream robotStateTopic;
  robotStateTopic << "/" << GetId() << "/robots_state";
  robotStateSub = nodeHandle->subscribe(robotStateTopic.str(), 1, &CKheperaIVRos::robotStateCallback, this);

  stringstream movebaseGoalTopic;
  movebaseGoalTopic << "/" << GetId() << "/move_base/current_goal";
  movebaseGoalSub = nodeHandle->subscribe(movebaseGoalTopic.str(), 1, &CKheperaIVRos::movebaseGoalCallback, this);

  // time
  time = ros::Time(0.0);

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcEncoder = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
  m_pcProximity = GetSensor<CCI_KheperaIVProximitySensor>("kheperaiv_proximity");
  m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
  m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  m_pcLIDAR = GetSensor<CCI_KheperaIVLIDARSensor>("kheperaiv_lidar");
  m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
}

void CKheperaIVRos::ControlStep()
{
  this->updateTime();
  this->publishLineOfSight();
  this->publishProximity();
  this->publishLIDAR();
  this->publishOdometry();
  this->debug(false);
  // Wait for any callbacks to be called.
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
}

void CKheperaIVRos::debug(bool debug)
{
  if (!debug)
  {
    return;
  }
  RLOG << time.toSec() << endl;
}

void CKheperaIVRos::updateTime()
{
  // updates rostime to sync all ros processes to argos clock
  // prevtime = time;
  time += ros::Duration(timestep);
  // timestep = time.toSec() - prevtime.toSec()
}

void CKheperaIVRos::publishLIDAR()
{
  sensor_msgs::LaserScan scan;
  scan.header.stamp = time;
  string frame = GetId() + "/base_footprint";
  scan.header.frame_id = frame;
  scan.angle_min = -KHEPERAIV_LIDAR_ANGLE_SPAN.GetValue() * 0.5;
  scan.angle_max = KHEPERAIV_LIDAR_ANGLE_SPAN.GetValue() * 0.5;
  scan.angle_increment = KHEPERAIV_LIDAR_ANGLE_SPAN.GetValue() / m_pcLIDAR->GetNumReadings();
  scan.time_increment = timestep;
  scan.scan_time = timestep;
  scan.range_min = KHEPERAIV_LIDAR_SENSORS_FAN_RADIUS + KHEPERAIV_LIDAR_SENSORS_RING_RANGE.GetMin();
  scan.range_max = KHEPERAIV_LIDAR_SENSORS_FAN_RADIUS + KHEPERAIV_LIDAR_SENSORS_RING_RANGE.GetMax();

  vector<float> lidarReadings;
  for (size_t j = 0; j < m_pcLIDAR->GetNumReadings(); ++j)
  {
    lidarReadings.push_back((float)m_pcLIDAR->GetReading(j) / 100);
  }
  scan.ranges = lidarReadings;
  laserScanPub.publish(scan);
}

void CKheperaIVRos::publishLineOfSight()
{
  // broadcast current robot to all LOS
  char nulls[2] = {'0', '0'};
  list_of_los_robots_.clear();
  uint8_t *empty = reinterpret_cast<uint8_t *>(&nulls);

  const char *id = GetId().c_str();
  size_t id_len = strlen(id);
  // Copy id to payload
  // strcpy(&payload, id);
  // memcpy(&payload,strlen(id))
  // Convert c_string to bytes
  uint8_t *data = reinterpret_cast<uint8_t *>(const_cast<char *>(id));
  // TODO verify id length is less than the message size
  CByteArray buff = CByteArray(data, id_len);
  buff.AddBuffer(empty, m_pcRABA->GetSize() - id_len);
  m_pcRABA->SetData(buff);

  // write all robot names within los to rosmsg
  const CCI_RangeAndBearingSensor::TReadings &packets = m_pcRABS->GetReadings();
  losList loslist;
  loslist.n = packets.size();
  for (size_t i = 0; i < packets.size(); ++i)
  {
    los losmsg;

    // I believe the packets are encoded in a non ASCII format.
    // Therefore, when converting to a string, we still see crazy characters.
    // Question: Where is the data from m_pcRABS->GetReadings() comming from?

    // argos::CByteArray packets[i]
    // Docs: https://www.argos-sim.info/api/a02302.php

    const unsigned char *data = packets[i].Data.ToCArray();
    // stringstream incoming_msg;
    // for(int ii = 0; ii<packets[i].Data.Size(); i++){
    //   incoming_msg << packets[i].Data[ii];
    //   if(packets[i].Data[ii] == '\n'){
    //     break;
    //   }
    // }
    std::string s(reinterpret_cast<const char *>(data), packets[i].Data.Size());
    s = s.substr(0,robot_name_length_);
    // TODO add parsing for name length
    list_of_los_robots_.push_back(s);

    losmsg.robotName = s; // incoming_msg.str();
    loslist.robots.push_back(losmsg);
  }
  losPub.publish(loslist);
}

void CKheperaIVRos::getListOfLOSRobots(std::vector<std::string>& list_of_los_robots)
{
  list_of_los_robots = list_of_los_robots_;
}

void CKheperaIVRos::getMotionVector(CVector3& cVelocity)
{
  cVelocity = cVelocity_;
}

std::string CKheperaIVRos::getRobotState()
{
  return robot_action_;
}

void CKheperaIVRos::getMovebaseGoalVector(CVector3& cMovebaseGoal)
{
  cMovebaseGoal = cMovebaseGoal_;
}

char const *CKheperaIVRos::parse_id_number(const std::string data)
{
  return data.c_str();
}

void CKheperaIVRos::publishProximity()
{
  /* Get readings from proximity sensor */
  const CCI_KheperaIVProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
  ProximityList proxList;
  proxList.n = tProxReads.size();
  for (size_t i = 0; i < proxList.n; ++i)
  {
    Proximity prox;
    prox.value = tProxReads[i].Value;
    prox.angle = tProxReads[i].Angle.GetValue();
    proxList.proximities.push_back(prox);
  }
  proximityPub.publish(proxList);
}

void CKheperaIVRos::publishOdometry()
{
  CRadians cX, cY, cZ;
  CCI_PositioningSensor::SReading sReadings;

  sReadings = m_pcPosition->GetReading();
  sReadings.Orientation.ToEulerAngles(cZ, cY, cX);
  // RLOG<<sReadings.Position.GetX()<<std::endl;
  // RLOG<<cZ.GetValue()<<std::endl;

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(cX.GetValue(), cY.GetValue(), cZ.GetValue() );
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(myQuaternion);

  /*
   * publish odom messages and TF transform
   */
  string header_frame_id = GetId() + "/odom";
  string child_frame_id = GetId() + "/base_footprint";

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = time;
  odom_trans.header.frame_id = header_frame_id;
  odom_trans.child_frame_id = child_frame_id;
  odom_trans.transform.translation.x = sReadings.Position.GetX();
  odom_trans.transform.translation.y = sReadings.Position.GetY();
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = quat_msg;
  odom_broadcaster->sendTransform(odom_trans);
  // RLOG << "publish transform" << endl;
  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = header_frame_id;
  odom.pose.pose.position.x = sReadings.Position.GetX();
  odom.pose.pose.position.y = sReadings.Position.GetY();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quat_msg;
  odom.child_frame_id = child_frame_id;
  odom.twist.twist.linear.x = odom_dx;
  odom.twist.twist.linear.y = odom_dy;
  odom.twist.twist.angular.z = odom_w;
  odomPub.publish(odom);
}

/****************************************/
/****************************************/

void CKheperaIVRos::cmdVelCallback(const geometry_msgs::Twist &twist)
{
  Real v = twist.linear.x * 100.0f;  // Forward speed
  Real w = twist.angular.z * 100.0f; // Rotational speed
  if (abs(w) < goStraightConstant)
  {
    w = 0.0;
  }
  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = (v - KHEPERAIV_BASE_RADIUS * w);// * KHEPERAIV_WHEEL_RADIUS;
  rightSpeed = (v + KHEPERAIV_BASE_RADIUS * w);// * KHEPERAIV_WHEEL_RADIUS;

  cVelocity_ = CVector3(3*twist.linear.x/5, 3*twist.angular.z/10, 0.1);

  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void CKheperaIVRos::robotStateCallback(const mdis_state_machine::RobotsState& robot_state)
{
  int state = robot_state.robot_state;
  if(robot_state_action_map_.count(state)>0)
    robot_action_ = robot_state_action_map_[state];
  else
    robot_action_ = "IRREGULAR_STATE_FOUND";
}

void CKheperaIVRos::movebaseGoalCallback(const geometry_msgs::PoseStamped& pose)
{
  geometry_msgs::Point position = pose.pose.position;
  cMovebaseGoal_ = CVector3(position.x, position.y, 0.1);
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CKheperaIVRos, "kheperaiv_ros_controller")
