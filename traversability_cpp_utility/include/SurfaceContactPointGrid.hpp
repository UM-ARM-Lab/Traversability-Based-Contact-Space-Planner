#ifndef SURFACECONTACTPOINTGRID_HPP
#define SURFACECONTACTPOINTGRID_HPP

// #include "Utilities.hpp"

class SurfaceContactPointGrid : public PointGrid
{
public:
    SurfaceContactPointGrid(float _min_x, float _max_x, float _min_y, float _max_y, float _resolution) : 
    PointGrid(_min_x, _max_x, _min_y, _max_y, _resolution){};

    float getScore(GridPositions2D position, ContactType type, Translation3D contact_direction, std::string feature_type);

    std::vector< std::vector<ContactPoint> > contact_point_list_;

private:

};

#endif