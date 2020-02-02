#include "Utilities.hpp"

bool ContactRegion::isInsideContactRegion(Translation3D point)
{
    return euclideanDistance3D(position_, point) <= radius_;
}

bool ContactRegion::isInsideContactRegion(Translation2D point)
{
    return euclideanDistance2D(projected_position_, point) <= radius_;
}