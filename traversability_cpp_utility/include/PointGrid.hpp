#ifndef POINTGRID_HPP
#define POINTGRID_HPP

// #include "Utilities.hpp"

// // OpenRAVE
// #include <openrave/plugin.h>

class PointGrid
{
public:
    PointGrid(float _min_x, float _max_x, float _min_y, float _max_y, float _resolution);

    void initializeParameters(float _min_x, float _max_x, float _min_y, float _max_y, float _resolution);

    GridIndices2D positionsToIndices(GridPositions2D position);
    GridPositions2D indicesToPositions(GridIndices2D indices);
    
    inline std::array<int,2> getDimensions(){return {dim_x_, dim_y_};}
    inline std::array<float,4> getBoundaries(){return {min_x_, max_x_, min_y_, max_y_};}
    inline std::array<float,2> getCenter(){return {center_x_, center_y_};}
    inline float getResolution(){return resolution_;}
    inline bool insideGrid(GridPositions2D positions){return (positions[0] >= min_x_ && positions[0] < max_x_ && positions[1] >= min_y_ && positions[1] < max_y_);}
    inline bool insideGrid(GridIndices2D indices){return (indices[0] >= 0 && indices[0] < dim_x_ && indices[1] >= 0 && indices[1] < dim_y_);}

    float getInterpolatedScore(std::array<float,4> scores, std::array<float,4> dist_to_boundaries);

protected:
    float resolution_;
    int dim_x_;
    int dim_y_;

    float min_x_;
    float max_x_;
    float min_y_;
    float max_y_;

    float center_x_;
    float center_y_;

};

#endif