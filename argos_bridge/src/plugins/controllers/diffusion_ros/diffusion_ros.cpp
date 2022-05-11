#include "diffusion_ros.h"

using namespace std;

CDiffusionRos::CDiffusionRos(): m_pcProximity(NULL),
                                m_cAlpha(10.0f),
                                m_fDelta(0.5f),
                                m_fWheelVelocity(2.5f),
                                m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                                                        ToRadians(m_cAlpha))
{
}

/****************************************/
/****************************************/

void CDiffusionRos::Init(TConfigurationNode &t_node)
{
  // Name and ROS
  controller_name = GetId();    
  CDiffusionRos::InitROS();

  // time
  previous_time = ros::Time(0.0);
  
  // Get sensor/actuator handles
  m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcEncoder   = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");
  m_pcProximity = GetSensor  <CCI_KheperaIVProximitySensor    >("kheperaiv_proximity");
  
  // parse argos config
  GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
  m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
  GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
  GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

  RLOG << controller_name << " diffusion initalized" << std::endl;
}

/****************************************/
/****************************************/

void CDiffusionRos::ControlStep()
{
  // calculate time diff
  ros::Time temp = ros::Time::now();
  ros::Duration time_step = temp - previous_time;
  previous_time = temp;

  // update odom
  CalculateOdometry(time_step);
  UpdateOdometryMessage();
  UpdateOdometryTF();

  // update diffusion control
  SetDiffusionControl();
  // ros spin
  ros::spinOnce();
}

/****************************************/
/****************************************/

void CDiffusionRos::SetDiffusionControl()
{
    // Get prox sensor readings
    const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    CVector2 cAccumulator;
    for(size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    /* If the angle of the vector is small enough and the closest obstacle
        * is far enough, continue going straight, otherwise curve a little
        */
    CRadians cAngle = cAccumulator.Angle();
    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta ) {
        /* Go straight */
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
    else {
        /* Turn, depending on the sign of the angle */
        if(cAngle.GetValue() > 0.0f) {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
        }
        else {
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
        }
   }
}

/****************************************/
/****************************************/

void CDiffusionRos::InitROS()
{
  // ROS node
  std::string name = "argos_bridge";
  char *argv = (char *)"";
  int argc = 0;
  ros::init(argc, &argv, name);
  nh = new ros::NodeHandle();
  
  // Odometry Publisher
  stringstream odomTopic;
  odomTopic << "/" << controller_name << "/odom";
  odom_pub = nh->advertise<nav_msgs::Odometry>(odomTopic.str(), 1);
  
  //tf broadcaster
  tf_broadcaster = new tf::TransformBroadcaster();
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
REGISTER_CONTROLLER(CDiffusionRos, "diffusion_ros_controller")