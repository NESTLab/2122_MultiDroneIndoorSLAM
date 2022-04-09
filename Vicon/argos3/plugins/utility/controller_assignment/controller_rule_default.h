#ifndef CONTROLLER_RULE_UNLIMITED_H
#define CONTROLLER_RULE_UNLIMITED_H

#include <argos3/plugins/utility/controller_assignment/controller_rule.h>
#include <argos3/plugins/utility/controller_assignment/controller_assigner.h>
#include <argos3/core/utility/datatypes/datatypes.h>

using namespace argos;

class CControllerRuleDefault : public CControllerRule{
public:
    void Init(TConfigurationNode& t_tree) override;

    std::string GetController() override;
};

#endif