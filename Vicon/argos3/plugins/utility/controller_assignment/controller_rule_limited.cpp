#include <controller_rule_limited.h>

void CControllerRuleLimited::Init(TConfigurationNode& t_tree){
    CControllerRule::Init(t_tree);
    GetNodeAttributeOrDefault(t_tree, "limit", m_unLimit, m_unLimit);
}

std::string CControllerRuleLimited::GetController(){
    if(m_unAllocated < m_unLimit){
        m_unAllocated++;
        return m_strController;
    }
    return "";
}

REGISTER_CONTROLLER_RULE(CControllerRuleLimited, "limited");
