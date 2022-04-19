#ifndef CONTROLLER_RULE_LIMIED_H
#define CONTROLLER_RULE_LIMIED_H

#include <argos3/plugins/utility/controller_assignment/controller_rule.h>
#include <argos3/plugins/utility/controller_assignment/controller_assigner.h>
#include <argos3/core/utility/datatypes/datatypes.h>

using namespace argos;

class CControllerRuleLimited : public CControllerRule{
public:
    std::string GetController() override;
    
    void Init(TConfigurationNode& t_tree) override;

private:
    UInt16 m_unLimit=0, m_unAllocated=0;
};

#endif