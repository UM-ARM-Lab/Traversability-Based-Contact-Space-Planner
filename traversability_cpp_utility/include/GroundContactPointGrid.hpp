#ifndef CONTACTPOINTGRID_HPP
#define CONTACTPOINTGRID_HPP

// #include "Utilities.hpp"

class GroundContactPointGrid : public PointGrid
{
public:
    GroundContactPointGrid(float _min_x, float _max_x, float _min_y, float _max_y, float _resolution) : 
    PointGrid(_min_x, _max_x, _min_y, _max_y, _resolution){};

    // float getScore(GridPositions2D position, ContactType type, Translation3D contact_direction);

    std::vector< std::vector< std::array<float,3> > > score_cell_list_;

private:

};

#endif