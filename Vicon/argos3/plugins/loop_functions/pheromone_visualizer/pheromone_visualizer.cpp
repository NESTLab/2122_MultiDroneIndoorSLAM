#include "pheromone_visualizer.h"

#include <argos3/plugins/loop_functions/master_loop_functions/master_loop_functions.h>

/****************************************/
/****************************************/

CPheromoneQTUserFunctions::CPheromoneQTUserFunctions() :
   m_cPheraMed(CSimulator::GetInstance().GetMedium<CPheromoneMedium>("pheromone"))
   {
    // get the grid size and calculate the square corners
    m_fGridSize = m_cPheraMed.GetCellSize();
    m_cSquarePoints.resize(4);
    m_cSquarePoints[1].Set(m_fGridSize,0);
    m_cSquarePoints[2].Set(m_fGridSize,m_fGridSize);
    m_cSquarePoints[3].Set(0,m_fGridSize);

    // set the orienation for the squares
    m_cOrientation.FromAngleAxis(CRadians::ZERO, CVector3::Z);
}

/****************************************/
/****************************************/

void CPheromoneQTUserFunctions::Init(TConfigurationNode& t_tree){
    m_bDrawDiffusion = false;
    GetNodeAttributeOrDefault(t_tree, "draw_diffusion", m_bDrawDiffusion, m_bDrawDiffusion);
}

/****************************************/
/****************************************/

void CPheromoneQTUserFunctions::DrawInWorld() {
    if(m_bDrawDiffusion)
        CDiffusionQTUserFunctions::DrawInWorld();
    const CPheromoneMedium::PheromoneMap& cPheromoneCells = m_cPheraMed.GetPheromoneMap();
    // offset is so the square is not in the same space as the floor 
    CVector3 cOffset(0,0,0.001);
    /* Go through all the cells and draw them */
    for(const auto &pair: cPheromoneCells){
        DrawPolygon(pair.first*m_fGridSize+cOffset,
                    m_cOrientation, 
                    m_cSquarePoints,
                    CColor(pair.second,0,0));
    }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CPheromoneQTUserFunctions, "pheromone_qtuser_functions")
