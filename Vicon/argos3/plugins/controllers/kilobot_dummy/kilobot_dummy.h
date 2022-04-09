#ifndef KILOBOT_DUMMY_H
#define KILOBOT_DUMMY_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKilobotDummy: public CCI_Controller {
public:

    CKilobotDummy(){}

   virtual ~CKilobotDummy() {}

   virtual void Init(TConfigurationNode& t_node){}

   virtual void ControlStep(){}

   virtual void Reset() {}

   virtual void Destroy() {}

protected:

};

#endif
