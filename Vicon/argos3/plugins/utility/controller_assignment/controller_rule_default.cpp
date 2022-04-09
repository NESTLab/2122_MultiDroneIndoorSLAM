#include <controller_rule_default.h>

void CControllerRuleDefault::Init(TConfigurationNode& t_tree){
    std::string strController;
    GetNodeAttribute(t_tree, "default", strController);
    m_strController = strController;
}

std::string CControllerRuleDefault::GetController(){
    return m_strController;
}

REGISTER_CONTROLLER_RULE(CControllerRuleDefault, "default")
