/*
 * AUTHOR: Peter Nikopoulos <peter@nikopoulos.net>
 *
 * Enables ROS Kheperaiv IV robot to perform diffusion exploration
 * Essentially replaces the /cmd_vel topics with diffusion logic
 *
 */

#ifndef DIFFUSION_ROS_H
#define DIFFUSION_ROS_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>
/* Definition of the kheperaiv lidar sensor */
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_lidar_sensor.h>
/* Definition of the kheperaiv measurements */
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_measures.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <string>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "math.h"

#include <../differential_drive/differential_drive.h>


using namespace argos;

class CDiffusionLidar : public CDiffusionRos 
{
public:
  CDiffusionRos();
  virtual ~CDiffusionRos() {};

  /**
   * @brief
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><footbot_ccw_wander_controller> section.
   */
  virtual void Init(TConfigurationNode &t_node);

  /**
   * @brief
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /**
   * @brief Starts rosnode for robot controller
   * 
   * @return void *
   */
  virtual void InitROS();

  /**
   * @brief inteprets lidar data at each control step and publishes an update
   * 
   * @return void *
   */

private:
  /*********************************************************************
   * ARGOS Sensors and Actuators
   ********************************************************************/
  /* Pointer to the Khepera IV p sensor */                                                                                                                         
  CCI_KheperaIVLIDARSensor* m_pcProximity;

  /*********************************************************************
   * ROS Publishers
  ********************************************************************/
  nav_msgs::Odometry odom_msg;
  ros::Publisher odom_pub;
};

#endif