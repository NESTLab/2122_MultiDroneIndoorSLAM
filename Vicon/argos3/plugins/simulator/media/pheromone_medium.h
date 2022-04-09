#ifndef PHEROMONE_MEDIUM_H
#define PHEROMONE_MEDIUM_H

#include <map>
#include <functional>
#include <cmath>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/medium/medium.h>
#include <argos3/core/utility/datatypes/datatypes.h>

using namespace argos;


// used for PheromoneMap allowing vectors to be indicies
struct VectorCompare
{
   bool operator() (const CVector3& lhs, const CVector3& rhs) const
   {
       if(lhs.GetX() == rhs.GetX()){
            if(lhs.GetY() == rhs.GetY()){
                return lhs.GetZ() < rhs.GetZ();
            }
            return lhs.GetY() < rhs.GetY();
       }
       return lhs.GetX() < rhs.GetX();
   }
};

class CPheromoneMedium : public CMedium {

public:
    typedef std::map<CVector3, UInt16, VectorCompare> PheromoneMap;

    typedef std::function<void(UInt16&)> PheromoneUpdater;

    void Init(TConfigurationNode& t_tree) override;

    void PostSpaceInit() override;
    
    void Reset() override;
    
    void Destroy() override;
    
    void Update() override;


    /**
     * Converts a real position to the cell position, this can be 
     * used for inserting elements into m_cPheromoneCells
     * @param  c_position   The real position.
     * @param  b_ceil       The coordinates rounds up.
     * @return The cell position.
     */
    CVector3 PositionToCellPosition(const CVector3& c_position, 
                                    const bool& b_floor=true) const;

    /**
     * Lays pheromone at the given position.
     * Internally PositionToCellPosition will be called, so the real position can be given.
     * @param c_position  The position where you want to lay pheromone.
     * @param un_strength The strength of the laid pheromone
     */
    void LayPheromone(const CVector3& c_position, const UInt16& un_strength);

    // TODO: use true and false for bool params
    /**
     * Reads the pheromone levels around the given position.
     * Internally PositionToCellPosition will be called, so the real position can be given.
     * @param c_position    The position around which to measure pheromone.
     * @param f_range       The distance of cells to be returned  
     * @param b_circle      If enabled, a circle filter is used. Not yet implemented.
     * @returned            A PheromoneMap of the local pheromone levels, with relative positions.
     */
    PheromoneMap ReadPheromone(const CVector3& c_position, const Real& f_range, 
                               const bool& b_circle=false) const;


    /**
     * @return The size of the cells
     */
    inline Real GetCellSize(){
        return m_fCellSize;
    }

    inline PheromoneMap GetPheromoneMap(){
        return m_cPheromoneCells;
    }

    /**
     * [SetLocalLevels description]
     * @param c_local_levels [description]
     * @param c_cell_key     [description]
     * @param c_position     [description]
     */
    void SetLocalLevels(CPheromoneMedium::PheromoneMap& c_local_levels,
                                      const CVector3& c_cell_key,
                                      const CVector3& c_position)const;

    /**
     * Converts an real coordinate to a cell coordinate.
     * @param  f_original_coordinate The original coordinate.
     * @param  b_ceil                Indicates that the coordinates should round up.
     * @return                       The cell coordinate.
     */
    inline Real ToCellDistance(const Real& f_original_coordinate, 
                                 const bool& b_floor = true) const{
        if(b_floor)
            return floor(f_original_coordinate/m_fCellSize);
        return f_original_coordinate/m_fCellSize;
    }

    inline PheromoneUpdater LinearDecay(const SInt32& un_step){
        return [un_step](UInt16& unLevel){if(unLevel > un_step) unLevel -= un_step; else unLevel = 0;};
    }

    inline PheromoneUpdater ExponentialDecay(const Real& f_decrease){
        return [f_decrease](UInt16& unLevel){unLevel *= f_decrease;};
    }

    inline const bool& IsThreeDimensions(){
        return m_bThreeDimensions;
    }

private:


    /** The dimension of the pheromone cell size */
    Real m_fCellSize;

    /** Whether or not laying more pheromone adds to or replaces the current amount. */
    bool m_bAdditive;

    /** Whether the z axis should be considered. */
    bool m_bThreeDimensions;

    /** Function to update pheromone levels for a cell value*/
    PheromoneUpdater m_cPheromoneUpdater;

    /** 
     * Keeps track of the pheromone values at certain positions. 
     * Do not write to the map directly, first use 
     * PositionToCellPosition to get valid cell positions.
     */
    PheromoneMap m_cPheromoneCells;
};

#endif
