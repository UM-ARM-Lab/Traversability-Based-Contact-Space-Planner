// #include "ContactPoint.hpp"
#include "Utilities.hpp"


float ContactPoint::getOrientationScore(Translation3D approaching_direction)
{
    Translation3D n = getNormal();
    float angular_deviation = (acos(n.dot(approaching_direction)) * RAD2DEG);

    if(angular_deviation < MAX_ANGULAR_DEVIATION)
    {
        return (MAX_ANGULAR_DEVIATION - tan(angular_deviation * DEG2RAD))/MAX_ANGULAR_DEVIATION;
    }
    else
    {
        return 0;
    }

}

float ContactPoint::getClearanceScore(ContactType type)
{
    float min_clearance, max_clearance;

    if(type == FOOT)
    {
        max_clearance = FOOT_RADIUS;
        min_clearance = FOOT_WIDTH/2.0;
    }
    else if(type == HAND)
    {
        max_clearance = HAND_RADIUS;
        min_clearance = HAND_WIDTH/2.0;
    }
    else
    {
        RAVELOG_ERROR("Function get_clearance_score: Invalid ContactType: %d.",type);
    }

    if(clearance_ < min_clearance)
    {
        return 0;
    }
    else if(clearance_ >= max_clearance)
    {
        return 1;
    }
    else
    {
        return (clearance_ - min_clearance) / (max_clearance-min_clearance);
    }

}

float ContactPoint::getTotalScore(ContactType type, Translation3D approaching_direction)
{
            // if(contact_type == 'foot'):
            // if(self.foot_contact_score is None):
            //     self.foot_contact_score = self.clearance_score(contact_type) * self.orientation_score(contact_direction)

            // return self.foot_contact_score
    return (getClearanceScore(type) * getOrientationScore(approaching_direction));
    // return getClearanceScore(type);
}