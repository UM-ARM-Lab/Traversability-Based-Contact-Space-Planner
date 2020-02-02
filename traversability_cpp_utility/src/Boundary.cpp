#include "Utilities.hpp"

Boundary::Boundary(Translation2D _point1, Translation2D _point2)
{
    end_points_ = std::make_pair(_point1, _point2);
    boundary_length_ = euclideanDistance2D(_point1, _point2);
    boundary_unit_vector_ = (_point2-_point1).normalized();
}

float Boundary::getDistance(Translation2D query_point)
{
    Translation2D vec =  query_point - end_points_.first;
    float proj = vec.dot(boundary_unit_vector_);

    if(proj > boundary_length_)
    {
        return euclideanDistance2D(query_point, end_points_.second);
    }
    else if(proj <= boundary_length_ && proj >= 0)
    {
        return euclideanDistance2D(query_point, end_points_.first + proj*boundary_unit_vector_);
    }
    else
    {
        return euclideanDistance2D(query_point, end_points_.first);
    }

}