/* Include the controller definition */
#include "khepera_diffusion.h"
/* Function definitions for XML parsing */
/* 2D vector definition */
#include <argos3/core/utility/logging/argos_log.h>
#include <algorithm>

/****************************************/
/****************************************/

void CKheperaDiffusion::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "hard_reset_angle_threshold", cAngle);
      HardTurnResetThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

CKheperaDiffusion::CKheperaDiffusion() :
   m_pcWheels(nullptr),
   m_pcProximity(nullptr),
   m_pcTrackedProximity(nullptr) {}

/****************************************/
/****************************************/

void CKheperaDiffusion::Init(TConfigurationNode& t_node) {
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_KheperaIVProximitySensor    >("kheperaiv_proximity"  );

   /* Wheel turning */
   m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
}

/****************************************/
/****************************************/

void CKheperaDiffusion::ControlStep() {
   CVector2 cGoalVector = GoalVector();
   if(cGoalVector.Length() > 1){
      cGoalVector = cGoalVector.Normalize();
   }

   SetWheelSpeedsFromVector(cGoalVector-ObstacleVector(), true);
}

/****************************************/
/****************************************/

CVector2 CKheperaDiffusion::ObstacleVector(){
   /* Get readings from proximity sensor */
   const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   CVector2 cAccumulator;

   for(size_t i = 0; i < m_unCheckSensors.size(); ++i) {
      UInt8 unSensor = m_unCheckSensors[i];
      cAccumulator += CVector2(tProxReads[unSensor].Value*m_fSensorsWeights[i], tProxReads[unSensor].Angle);
   }

   // Dividing by at most 3 because thats the most sensors can reasonably see durring a diffusion
   cAccumulator /= 3;

   return cAccumulator;
}
/****************************************/
/****************************************/

void CKheperaDiffusion::SetWheelSpeedsFromVector(const CVector2& c_heading, const bool& b_diffusion) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed;

   fBaseAngularWheelSpeed = b_diffusion ? m_sWheelTurningParams.MaxSpeed : 
                            Min<Real>(fHeadingLength*10, m_sWheelTurningParams.MaxSpeed);

   // LOG << "Current state: " << m_sWheelTurningParams.TurningMechanism
   //     << " Heading angle: " << cHeadingAngle
   //     << " Heading length:" << fHeadingLength
   //     << std::endl;

   /* Turning state switching conditions */
   if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
      /* No Turn, heading angle very small */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
   }
   else if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
      /* Hard Turn, heading angle very large */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
   }
   else if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN &&
           Abs(cHeadingAngle) <= m_sWheelTurningParams.HardTurnResetThreshold){
      /* Soft Turn */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
   }
   else if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN &&
           Abs(cHeadingAngle) > m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
      /* Soft Turn, heading angle in between the two cases */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
   }

   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }

      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }

      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
      default:
         fSpeed1 = 0;
         fSpeed2 = 0;
         LOG << "";
         break;
   }

   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
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
REGISTER_CONTROLLER(CKheperaDiffusion, "khepera_diffusion_controller")
