#ifndef CONTACTREGION_HPP
#define CONTACTREGION_HPP

// #include "Utilities.hpp"

// // OpenRAVE
// #include <openrave/plugin.h>

class ContactRegion
{

public:
	ContactRegion(Translation3D _position, Translation2D _projected_position, Translation3D _normal, float _radius):
    position_(_position),
    projected_position_(_projected_position),
    normal_(_normal),
    radius_(_radius) {};

    ContactRegion(ContactPoint _cp):
    position_(_cp.getPosition()),
    projected_position_(_cp.getProjectedPosition()),
    normal_(_cp.getNormal()),
    radius_(_cp.getClearance()) {};

	inline Translation3D getPosition() const { return position_; }
    inline Translation2D getProjectedPosition() const { return projected_position_; }
    inline Translation3D getNormal() const {return normal_;}
    inline float getRadius() const {return radius_;}

    bool isInsideContactRegion(Translation3D point);
    bool isInsideContactRegion(Translation2D point);

private:
	Translation3D position_;
    Translation2D projected_position_;
    Translation3D normal_;
    float radius_;

};

#endif