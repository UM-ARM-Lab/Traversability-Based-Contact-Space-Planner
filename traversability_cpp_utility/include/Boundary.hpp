#ifndef BOUNDARY_HPP
#define BOUNDARY_HPP

// #include "Utilities.hpp"

// // OpenRAVE
// #include <openrave/plugin.h>

class Boundary
{

public:
	Boundary(Translation2D _point1, Translation2D _point2);

    float getDistance(Translation2D query_point);
    inline std::pair <Translation2D,Translation2D> getEndPoints() const { return end_points_; }
    
private:
	std::pair <Translation2D,Translation2D> end_points_;
    Translation2D boundary_unit_vector_;
    float boundary_length_;

};

#endif