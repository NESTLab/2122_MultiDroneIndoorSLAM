#include "controller_assigner.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/plugins/dynamic_loading.h>

void CControlerAssigner::Init(TConfigurationNode& t_tree){
    /* Cycle through the robot types */
    TConfigurationNodeIterator itType;
    for(itType = itType.begin(&t_tree);
        itType != itType.end();
        ++itType) {
        CControlerAssigner::ParseRobotSubtree(*itType);
    }
}

/****************************************/
/****************************************/

void CControlerAssigner::Destroy(){
    // clear the map
    for(auto& pair: m_tControllers){
        // delete the pointer
        for(auto& tRule: pair.second){
            delete tRule.cRule;
        }
        m_tControllers.erase(pair.first);
    }
}

/****************************************/
/****************************************/

void CControlerAssigner::ParseRobotSubtree(TConfigurationNode& t_tree){
    TPControllerRules tRules;

    TConfigurationNodeIterator itRule;
    for(itRule = itRule.begin(&t_tree);
        itRule != itRule.end();
        ++itRule) {
        tRules.push_back(CControlerAssigner::ParseRule(*itRule));
    }

    std::sort(tRules.rbegin(), tRules.rend());

    TPriorityControllerRule tPRule; 
    tPRule.cRule = CFactory<CControllerRule>::New("default");
    tPRule.cRule->Init(t_tree);
    tRules.push_back(tPRule);

    m_tControllers[t_tree.Value()] = tRules;
}

/****************************************/
/****************************************/

TPriorityControllerRule CControlerAssigner::ParseRule(TConfigurationNode& t_tree){
    TPriorityControllerRule tPRule;
    std::string strLibrary;

    GetNodeAttributeOrDefault(t_tree, "library", strLibrary, strLibrary);
    if(! strLibrary.empty()){
        CDynamicLoading::LoadLibrary(strLibrary);
    }

    GetNodeAttributeOrDefault(t_tree, "priority", tPRule.unPriority, 0);

    tPRule.cRule = CFactory<CControllerRule>::New(t_tree.Value());
    tPRule.cRule->Init(t_tree);

    return tPRule;
}

/****************************************/
/****************************************/

std::string CControlerAssigner::GetController(std::string str_robot_type){
    std::string controller;
	for(TPriorityControllerRule &rule: m_tControllers[str_robot_type]){
        controller = rule.cRule->GetController();
        if(!controller.empty()){
            break;
        }
    }
    return controller;
}
