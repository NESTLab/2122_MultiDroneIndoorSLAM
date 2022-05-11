#include "differential_drive.h"

using namespace std;

CDifferentialDrive::CDifferentialDrive() : m_pcWheels(NULL),
                                           m_pcEncoder(NULL)
{
}

/****************************************/
/****************************************/

void CDifferentialDrive::Init(TConfigurationNode &t_node)
{
  // Name and ROS
  controller_name = GetId();    
  CDifferentialDrive::InitROS();

  // time
  previous_time = ros::Time(0.0);
  
  // Get sensor/actuator handles
  m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcEncoder   = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");
  
  RLOG << controller_name << "initlaized" << std::endl;
}

/****************************************/
/****************************************/

void CDifferentialDrive::ControlStep()
{
  // calculate time diff
  ros::Time temp = ros::Time::now();
  ros::Duration time_step = temp - previous_time;
  previous_time = temp;

  // update odom
  CalculateOdometry(time_step);
  UpdateOdometryMessage();
  UpdateOdometryTF();

  // ros spin
  ros::spinOnce();
}

/****************************************/
/****************************************/

void CDifferentialDrive::InitROS()
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

  // Command Velocity Subsriber
  stringstream cmdVelTopic;
  cmdVelTopic << "/" << controller_name << "/cmd_vel";
  cmd_vel_sub = nh->subscribe(cmdVelTopic.str(), 1, &CDifferentialDrive::CommandVelocityCallback, this);
  
  //tf broadcaster
  tf_broadcaster = new tf::TransformBroadcaster();
}

/****************************************/
/****************************************/

void CDifferentialDrive::InitOdometry()
{
  odom_header_frame_id = "odom";
  odom_child_frame_id = "base_footprint";
}

/****************************************/
/****************************************/

void CDifferentialDrive::CommandVelocityCallback(const geometry_msgs::Twist &twist)
{
  Real v = twist.linear.x * 100.0f * speed_scaler;  // Forward speed | mult 100 to go from m to cm
  Real w = twist.angular.z * 100.0f * speed_scaler; // Rotational speed
  if (abs(w) < goStraightConstant){
    w = 0.0;
  }
  Real leftSpeed = (v - KHEPERAIV_BASE_RADIUS * w) * KHEPERAIV_WHEEL_RADIUS;
  Real rightSpeed = (v + KHEPERAIV_BASE_RADIUS * w) * KHEPERAIV_WHEEL_RADIUS;
  RLOG << controller_name << std::endl;
  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

double* CDifferentialDrive::ReadMotorEncoders()
{
  double* last_velocity = new double[WHEEL_NUM];
  last_velocity[LEFT] = this->m_pcEncoder->GetReading().VelocityLeftWheel * KHEPERAIV_WHEEL_RADIUS / 100.0f; // div 100 to go from cm to m
  last_velocity[RIGHT] = this->m_pcEncoder->GetReading().VelocityRightWheel * KHEPERAIV_WHEEL_RADIUS / 100.0f;
  return last_velocity;
}

/****************************************/
/****************************************/

void CDifferentialDrive::CalculateOdometry(ros::Duration timeDiff)
{
  double* last_velocity = ReadMotorEncoders();
  double timestep = timeDiff.sec;
  /*
  * Update forward kinematic model of differental drive robot.
  */
  if (abs(last_velocity[RIGHT] - last_velocity[LEFT]) < goStraightConstant)
  {
    // Vl = Vr # drive in a straight line
    odom_vel[Z]  = 0.0;
    odom_pos[Z]  = odom_pos[Z];

    odom_vel[X]  = last_velocity[LEFT] * cos(odom_pos[Z]);
    odom_vel[Y]  = last_velocity[LEFT] * sin(odom_pos[Z]);
    odom_pos[X] += last_velocity[LEFT] * cos(odom_pos[Z]) * timestep;
    odom_pos[Y] += last_velocity[LEFT] * sin(odom_pos[Z]) * timestep;
  }
  else
  {
    double R = (KHEPERAIV_BASE_RADIUS / 100.0f) * ((last_velocity[LEFT] + last_velocity[RIGHT]) / (last_velocity[RIGHT] - last_velocity[LEFT]));

    odom_vel[Z] = (last_velocity[RIGHT] - last_velocity[LEFT]) / (KHEPERAIV_BASE_RADIUS * 2 / 100.0f);
    
    double ICCx = odom_pos[X] - R * sin(odom_pos[Z]);
    double ICCy = odom_pos[Y] + R * cos(odom_pos[Z]);
    double PREVx = odom_pos[X];
    double PREVy = odom_pos[Y];

    odom_pos[X] = (odom_pos[X] - ICCx) * cos(odom_vel[Z] * timestep) - (odom_pos[Y] - ICCy) * sin(odom_vel[Z] * timestep) + ICCx;
    odom_vel[X] = odom_pos[X] - PREVx / timestep;
    
    odom_pos[Y] = (odom_pos[X] - ICCx) * sin(odom_vel[Z] * timestep) + (odom_pos[Y] - ICCy) * cos(odom_vel[Z] * timestep) + ICCy;
    odom_vel[Y] = odom_pos[Y] - PREVy / timestep;
    
    odom_pos[Z] += odom_vel[Z] * timestep;
  }

  delete last_velocity;
}

/****************************************/
/****************************************/

void CDifferentialDrive::UpdateOdometryMessage()
{
  odom_msg.header.stamp = previous_time;
  odom_msg.header.frame_id = odom_header_frame_id;
  odom_msg.child_frame_id = odom_child_frame_id;

  odom_msg.pose.pose.position.x = odom_pos[X];
  odom_msg.pose.pose.position.y = odom_pos[Y];
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pos[Z]);

  odom_msg.twist.twist.linear.x = odom_vel[X];
  odom_msg.twist.twist.linear.y = odom_vel[Y];
  odom_msg.twist.twist.angular.z = odom_vel[Z];

  odom_pub.publish(odom_msg);
}

/****************************************/
/****************************************/

void CDifferentialDrive::UpdateOdometryTF()
{
  odom_tf.header.stamp = previous_time;
  odom_tf.header.frame_id = odom_header_frame_id;
  odom_tf.child_frame_id = odom_child_frame_id;

  odom_tf.transform.translation.x = odom_pos[X];
  odom_tf.transform.translation.y = odom_pos[Y];
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(odom_pos[Z]);

  tf_broadcaster->sendTransform(odom_tf);
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
REGISTER_CONTROLLER(CDifferentialDrive, "differential_drive_controller")