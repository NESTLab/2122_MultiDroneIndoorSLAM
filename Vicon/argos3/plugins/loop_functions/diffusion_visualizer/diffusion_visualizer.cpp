#include "diffusion_visualizer.h"
#include <argos3/core/control_interface/ci_controller.h>

#include <argos3/plugins/controllers/khepera_diffusion/khepera_diffusion.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>

/****************************************/
/****************************************/

void CDiffusionQTUserFunctions::DrawInWorld() {
    CSpace::TMapPerType* m_ptKheMap;
    try{
        m_ptKheMap = &(CSimulator::GetInstance().GetSpace().GetEntitiesByType("kheperaiv"));
    }catch(CARGoSException& ex){
        LOGERR << "Issue getting entities by type 'Khepera'" <<std::endl;
        return;
    }

    for(const auto& it: *m_ptKheMap) {

        /* Create a pointer to the current khepera and its controller*/
        CKheperaIVEntity* pcKE = any_cast<CKheperaIVEntity*>(it.second);
        CCI_Controller* pcController = &(pcKE->GetControllableEntity().GetController());
        
        CVector3 cPosition = pcKE->GetEmbodiedEntity().GetOriginAnchor().Position;
        cPosition.SetZ(0);
        CQuaternion cROrientation = pcKE->GetEmbodiedEntity().GetOriginAnchor().Orientation;
        
        CRadians cX, cY, cZ;
        cROrientation.ToEulerAngles(cZ, cY, cX);

        CVector3 cOffsetPosition = cPosition;
        cOffsetPosition.SetZ(.08);

        //Display its ID
        DrawText(cPosition, pcKE->GetId(), CColor::BLUE);
        //Draw forward direction
        DrawOffsetRay(cOffsetPosition, cOffsetPosition+CVector3(CVector3::X).RotateZ(cZ)*.15, CColor::BLACK);   

        auto pcFController = dynamic_cast<CKheperaDiffusion*>(pcController);
        if(pcFController != nullptr){

            // LOG << "ROrientation:" << cROrientation << "|cX:" << cX 
            //     << "|cY:" << cY  <<"|cZ:" << cZ << std::endl; 

            CVector3 cObstacleDelta = 
                Vector2ToVector3(pcFController->ObstacleVector().Rotate(cZ));

            CVector3 cGoal = 
                Vector2ToVector3(pcFController->GoalVector().Rotate(cZ));

            cGoal += cOffsetPosition;

            //Draw pole at robot center
            DrawRay(CRay3(cPosition, cOffsetPosition), CColor::BLACK);
            //Draw Obstacle vector
            DrawOffsetRay(cOffsetPosition, cOffsetPosition-cObstacleDelta, CColor::BLUE);
            //Draw Goal vector
            DrawOffsetRay(cOffsetPosition, cGoal, CColor::GREEN);
            //Draw sum vector
            DrawOffsetRay(cOffsetPosition, cGoal-cObstacleDelta, CColor::ORANGE);
            //Draw obstacle delta vector
            DrawOffsetRay(cGoal, cGoal-cObstacleDelta, CColor::YELLOW);
        }
    }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CDiffusionQTUserFunctions, "diffusion_qtuser_functions")
