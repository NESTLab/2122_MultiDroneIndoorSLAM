#ifndef CONTROLLER_ASSIGNER_H
#define CONTROLLER_ASSIGNER_H

#include <vector>
#include <map>
#include <algorithm>

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/utility/controller_assignment/controller_rule.h>
#include <argos3/plugins/utility/controller_assignment/controller_rule_unlimited.h>
#include <argos3/plugins/utility/controller_assignment/controller_rule_limited.h>

using namespace argos;

// Data structure for holding a single rule unPriority denotes how important 
// a rule is and what presidence it takes. Priority defaults to 0, with higher 
// rules having higher priority and being executed first. 
// The largest value for priority is 254.
// Rules with the same priority do not have their order garenteed and 
// code that relies on such an ordering is considered buggy.  
typedef struct SPriorityControllerRule{
    CControllerRule* cRule;
    UInt8 unPriority;
    bool operator < (const SPriorityControllerRule& sOther) const{
        return (unPriority < sOther.unPriority);
    }
} TPriorityControllerRule;

typedef std::vector<TPriorityControllerRule> TPControllerRules;
typedef std::map<std::string, TPControllerRules> TPControllerMap;

class CControlerAssigner{
public:
    void Init(TConfigurationNode& t_tree);

    void Destroy();

    void ParseRobotSubtree(TConfigurationNode& t_tree);

    TPriorityControllerRule ParseRule(TConfigurationNode& t_tree);

    std::string GetController(std::string str_robot_type);

private:
    TPControllerMap m_tControllers;

};

#endif
