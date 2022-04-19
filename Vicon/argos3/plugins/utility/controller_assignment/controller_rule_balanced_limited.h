#ifndef CONTROLLER_RULE_BALANCED_LIMIED_H
#define CONTROLLER_RULE_BALANCED_LIMIED_H

#include <argos3/plugins/utility/controller_assignment/controller_rule.h>

class CControllerRuleBalancedLimited : public CControllerRule{
public:
    CControllerRuleBalancedLimited(TRobotControllers str_controllers, UInt16 un_limit);

    virtual std::string GetController();

private:
    UInt16 m_unLimit;
    TRobotControllers strControllers;
};

#endif