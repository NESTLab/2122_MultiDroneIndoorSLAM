#ifndef PC_LOOP_FUNCTIONS_H
#define PC_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <buzz/argos/buzz_controller.h>


using namespace argos;

class CPCLoopFunctions : public CLoopFunctions {

public:


   void Init(TConfigurationNode& t_tree) override;

   void Reset() override;

   void Destroy() override;

   void PreStep() override;

   void PostStep() override;

   inline std::map<std::string, CVector2> GetTargetPos(){
      return m_tTargetPos;
   }

   inline std::map<std::string, CVector2> GetDepotPos(){
      return m_tDepotPos;
   }

   inline std::map<std::string, UInt32> GetTargetUpdated(){
      return m_tTargetUpdated;
   }

private:
   void InitVMs();

   bool GetGoal(buzzvm_t t_buzz_vm, std::string& strGoal, CVector2& cGoal);

   bool GetTolerance(buzzvm_t t_buzz_vm, Real& fTolerance);

   buzzvm_state TablePut(buzzvm_t t_buzzVM,
                         buzzobj_t t_table,
                         const std::string& str_key,
                         buzzobj_t t_obj);


   buzzvm_state TablePut(buzzvm_t t_buzzVM,
                         buzzobj_t t_table,
                         const std::string& str_key,
                         Real f_value);

   buzzvm_state TablePut(buzzvm_t t_buzzVM,
                         buzzobj_t t_table,
                         const std::string& str_key,
                         std::string str_value);

   buzzvm_state TablePut(buzzvm_t t_buzzVM,
                         buzzobj_t t_table,
                         const std::string& str_key,
                         const CVector2& c_vec);

   std::string m_strOutputFile;
   std::ofstream m_cOutput;
   // map of position and nearest depot for each target
   std::map<std::string, CVector2> m_tTargetPos;
   std::map<std::string, std::string> m_tTargetDepot;
   std::map<std::string, UInt32> m_tTargetUpdated;
   std::map<std::string, CVector2> m_tDepotPos;

   std::vector<CBuzzController*> m_tControllers;

};

#endif
