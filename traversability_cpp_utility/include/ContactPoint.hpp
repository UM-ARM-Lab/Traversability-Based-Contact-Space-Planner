#ifndef CONTACTPOINT_HPP
#define CONTACTPOINT_HPP

// #include "Utilities.hpp"

// // OpenRAVE
// #include <openrave/plugin.h>

class ContactPoint
{

public:
	ContactPoint(Translation3D _position, Translation2D _projected_position, Translation3D _normal, float _clearance, bool _feasible):
    position_(_position), 
    projected_position_(_projected_position),
    normal_(_normal),
    clearance_(_clearance),
    feasible_(_feasible),
    convered_(false) {};

	inline Translation3D getPosition() const { return position_; }
    inline Translation2D getProjectedPosition() const { return projected_position_; }
    inline Translation3D getNormal() const {return normal_;}
    inline float getClearance() const { return clearance_; }
    inline bool isFeasible() const {return feasible_;}

    float getOrientationScore(Translation3D approaching_direction);
    float getClearanceScore(ContactType type);
    float getTotalScore(ContactType type, Translation3D approaching_direction);

    bool operator < (const ContactPoint& cp) const
    {
        return (clearance_ < cp.getClearance());
    }

    inline void setClearance(float c){clearance_ = c;}

    bool feasible_;
    bool convered_;

private:
	Translation3D position_;
    Translation2D projected_position_;
    Translation3D normal_;
    float clearance_;

};

#endif