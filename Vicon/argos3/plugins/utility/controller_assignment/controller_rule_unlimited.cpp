#include <controller_rule_unlimited.h>

void CControllerRuleUnlimited::Init(TConfigurationNode& t_tree){
    CControllerRule::Init(t_tree);
}

std::string CControllerRuleUnlimited::GetController(){
    return m_strController;
}

REGISTER_CONTROLLER_RULE(CControllerRuleUnlimited, "unlimited")
