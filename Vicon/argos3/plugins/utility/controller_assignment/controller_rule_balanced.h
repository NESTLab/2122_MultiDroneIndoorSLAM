#ifndef CONTROLLER_RULE_BALANCED_H
#define CONTROLLER_RULE_BALANCED_H

#include <argos3/plugins/utility/controller_assignment/controller_rule.h>
#include <argos3/plugins/utility/controller_assignment/controller_assigner.h>

using namespace argos;

class CControllerRuleBalanced : public CControllerRule{
public:
    CControllerRuleBalanced(TRobotControllers str_controllers);

    virtual std::string GetController();

private:
    TRobotControllers strControllers;
};

#endif