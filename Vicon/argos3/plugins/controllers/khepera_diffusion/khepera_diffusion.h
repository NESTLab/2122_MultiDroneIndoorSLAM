/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the khepera.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef KHEPERA_DIFFUSION_H
#define KHEPERA_DIFFUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the khepera proximity sensor */
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>

#include <argos3/core/utility/math/vector2.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKheperaDiffusion : public CCI_Controller {

public:

     struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism = NO_TURN;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      CRadians HardTurnResetThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed = 0;

      void Init(TConfigurationNode& t_tree);
   };

public:

   /* Class constructor. */
   CKheperaDiffusion();

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><khepera_diffusion_controller> section.
    */
   void Init(TConfigurationNode& t_node) override;

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   void ControlStep() override;

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   void Reset() override{}


   virtual CVector2 ObstacleVector();

   virtual CVector2 GoalVector(){
      return m_cDiffusionGoal;
   };

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}


  void SetWheelSpeedsFromVector(const CVector2& c_heading, const bool& b_diffusion=false);

protected:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the khepera proximity sensor */
   CCI_KheperaIVProximitySensor* m_pcProximity;

   CCI_KheperaIVProximitySensor* m_pcTrackedProximity;

   // which sensors to check
   const std::vector<UInt8> m_unCheckSensors =  {0, 1, 2, 6, 7};
   const std::vector<Real> m_fSensorsWeights =  {1, 1.5, 1, 1.5, 1};
   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><khepera_diffusion_controller> section.
    */

    /* The turning parameters. */
    SWheelTurningParams m_sWheelTurningParams;

    CVector2 m_cDiffusionGoal = CVector2::X*.1;

};

#endif
