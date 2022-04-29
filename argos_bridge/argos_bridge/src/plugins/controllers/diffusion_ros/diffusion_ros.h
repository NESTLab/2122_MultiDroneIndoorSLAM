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

class CDiffusionRos : public CDifferentialDrive 
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
   * @brief reads the proximity sensors and updates motor velocity to avoid obsticles
   * 
   * @return void *
   */
  virtual void SetDiffusionControl();

private:
  /*********************************************************************
   * ARGOS Sensors and Actuators
   ********************************************************************/
  /* Pointer to the Khepera IV proximity sensor */                                                                                                                         
  CCI_KheperaIVProximitySensor* m_pcProximity;

  /*********************************************************************
   * robot params
   ********************************************************************/
  /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;
};

#endif