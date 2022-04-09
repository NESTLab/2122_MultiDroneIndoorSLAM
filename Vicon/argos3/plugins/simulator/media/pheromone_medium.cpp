#include "pheromone_medium.h"
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

void CPheromoneMedium::Init(TConfigurationNode& t_tree){
    CMedium::Init(t_tree);

    std::string strDecayOption("linear");

    GetNodeAttributeOrDefault(t_tree, "cell_size", m_fCellSize, 0.1);
    GetNodeAttributeOrDefault(t_tree, "additive", m_bAdditive, false);
    GetNodeAttributeOrDefault(t_tree, "3d", m_bThreeDimensions, false);
    GetNodeAttributeOrDefault(t_tree, "decay_function", strDecayOption, strDecayOption);

    if(strDecayOption == "linear") {
        SInt32 nStep;
        GetNodeAttributeOrDefault(t_tree, "step", nStep, 1);
        nStep = abs(nStep);
        m_cPheromoneUpdater = LinearDecay(nStep); 
    }else if(strDecayOption == "exponential"){
        Real fDecrease;
        GetNodeAttributeOrDefault(t_tree, "step", fDecrease, .9);
        m_cPheromoneUpdater = ExponentialDecay(fDecrease);
    }
    else{
        THROW_ARGOSEXCEPTION("Unknown update method \"" << strDecayOption 
           << "\" for updating pheromone levels.");
    }

}

/****************************************/
/****************************************/

void CPheromoneMedium::PostSpaceInit(){

}

/****************************************/
/****************************************/

void CPheromoneMedium::Reset(){
    m_cPheromoneCells.clear();
}

/****************************************/
/****************************************/

void CPheromoneMedium::Destroy(){

}

/****************************************/
/****************************************/

CVector3 CPheromoneMedium::PositionToCellPosition(const CVector3& c_position, 
   const bool& b_floor) const{

    CVector3 cCellPosition;
    Real fZ = m_bThreeDimensions ? ToCellDistance(c_position.GetZ(), b_floor) 
    : 0;

    cCellPosition.SetX(ToCellDistance(c_position.GetX(), b_floor));
    cCellPosition.SetY(ToCellDistance(c_position.GetY(), b_floor));
    cCellPosition.SetZ(fZ);
    return cCellPosition;
}

/****************************************/
/****************************************/

void CPheromoneMedium::Update(){
    for(auto &pair: m_cPheromoneCells){
        // update pheromone levels using callback chosen by Init
        m_cPheromoneUpdater(pair.second);
        // if pheromone level is 0 remove from map to make iteration quicker
        if(pair.second == 0){
            m_cPheromoneCells.erase(pair.first);
        }
    }
}

/****************************************/
/****************************************/

void CPheromoneMedium::LayPheromone(const CVector3& c_position, 
    const UInt16& un_strength){
    CVector3 cPosition = PositionToCellPosition(c_position);
    UInt16 unCurrLevel = m_bAdditive ? m_cPheromoneCells[cPosition] : 0;
    m_cPheromoneCells[cPosition] = unCurrLevel + un_strength;
}

/****************************************/
/****************************************/

inline void GetSearchRange(const UInt16& un_adj_range, const Real& f_range_offset,
  const Real& f_pos_offset, SInt16& un_min, SInt16& un_max){
    un_min = -un_adj_range;
    un_min -= (f_range_offset-f_pos_offset)>1 ? 1 : 0;
    un_max = un_adj_range;
    un_max += (f_range_offset+f_pos_offset)>1 ? 1 : 0;
}

/****************************************/
/****************************************/

void CPheromoneMedium::SetLocalLevels(CPheromoneMedium::PheromoneMap& c_local_levels,
    const CVector3& c_cell_key,
    const CVector3& c_position)const{

    CPheromoneMedium::PheromoneMap::const_iterator fKeyPair;

    // check to see if the cell has any pheromone
    fKeyPair = m_cPheromoneCells.find(c_cell_key);
    if(fKeyPair != m_cPheromoneCells.end()){
        // set the transorm the cordinates to meters and 
        // translate it to be releative to the robot position
        CVector3 CLocalPosition = c_cell_key*m_fCellSize - c_position;
        // Set the entry in the local map
        c_local_levels[CLocalPosition] = fKeyPair->second;
    }
}

/****************************************/
/****************************************/


CPheromoneMedium::PheromoneMap CPheromoneMedium::ReadPheromone(const CVector3& c_position, 
    const Real& f_range, 
    const bool& b_circle)const{
    CPheromoneMedium::PheromoneMap cLocalLevels;
    
    // convert from meters to cell values
    //
    const CVector3 cAdjPosition(PositionToCellPosition(c_position));
    const UInt16 fAdjRange(ToCellDistance(f_range));
    
    // get 
    const CVector3 cPosOffset(cAdjPosition-PositionToCellPosition(c_position, false));
    const Real fRangeOffset(fabs(fAdjRange-ToCellDistance(f_range, false)));

    CVector3 cCellKey;

    SInt16 nX=0, nXMin=0, nXMax=0, nY=0, nYMin=0, nYMax=0, nZ=0, nZMin=0, nZMax=0;

    // get the x, y, z range
    GetSearchRange(fAdjRange, fRangeOffset, cPosOffset.GetX(), nXMin, nXMax);
    GetSearchRange(fAdjRange, fRangeOffset, cPosOffset.GetY(), nYMin, nYMax);
    if(m_bThreeDimensions)
        GetSearchRange(fAdjRange, fRangeOffset, cPosOffset.GetZ(), nZMin, nZMax);

    // for all the cells in the range, set the local levels
    for(nX = nXMin; nX < nXMax; ++nX){
        for(nY = nYMin; nY < nYMax; ++nY){
            if(m_bThreeDimensions){
                for(nZ = nZMin; nZ < nZMax; ++nZ){
                    cCellKey.Set(nX,nY,nZ);
                    cCellKey += cAdjPosition;
                    SetLocalLevels(cLocalLevels, cCellKey, c_position);
                }
            }else{
                cCellKey.Set(nX,nY,0);
                cCellKey += cAdjPosition;
                SetLocalLevels(cLocalLevels, cCellKey, c_position);
            }
        }
    }

    return cLocalLevels;
}
/****************************************/
/****************************************/

REGISTER_MEDIUM(CPheromoneMedium,
 "pheromone",
 "Chris Cormier [ccormier@wpi.edu]",
 "0.1",
 "Medium that manages pheromone trails.",
 "This medium tracks pheromone laid by the pheromone_acuator\n"
 "and is required if you intend to use the pheromone sensor.\n\n"
 "REQUIRED XML CONFIGURATION\n\n"
 "<pheromone id=\"pheromone\" />\n\n"
 "OPTIONAL XML ATTRIBUTES\n\n"
 "cell_size - The size in meters of the pheromone cells, defaults to .1\n"
 "additive - Whether new pheromone is added to the previous value, defaults to false\n"
 "3d - Whether Voxels are used for pheromone cells, defaults to false\n"
 "decay_function - [linear|exponential] How pheromone levels decrease , defaults to linear\n"
 "step - how quickly pheromone levels decrease over time\n"
 "\t when decay_function is linear, step is an integer, defaults to 10"
 "\t when decay_function is exponential, step is a float, defaults to .9",
 "Under development"
 );
