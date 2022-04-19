#ifndef CONTROLLER_RULE_H
#define CONTROLLER_RULE_H

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/plugins/factory.h>

using namespace argos;

class CControllerRule{
public:

    virtual ~CControllerRule() = default;
    /**
     * Gives the controller for the robot according to the rule.
     * May return an empty string if the rule is not currently applicable.
     * @return          The name of the controller to use. 
     */    
    virtual std::string GetController() = 0;

    virtual inline void Init(TConfigurationNode& t_tree) {
        std::string strController;
        GetNodeAttribute(t_tree, "controller", strController);
        m_strController = strController;

    };

protected:
    std::string m_strController;
};

typedef CFactory<CControllerRule> TFactoryControllerRule;

#define REGISTER_CONTROLLER_RULE(CLASSNAME, LABEL) REGISTER_SYMBOL(CControllerRule, CLASSNAME, LABEL,"","","","","")

#endif
