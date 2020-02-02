#include "Utilities.hpp"
// #include "SurfaceContactPointGrid.hpp"



float SurfaceContactPointGrid::getScore(GridPositions2D position, ContactType contact_type, Translation3D contact_direction, std::string feature_type)
{
    if(position[0] >= max_x_ || position[0] < min_x_ || position[1] >= max_y_ || position[1] < min_y_)
    {
        return 0;
    }

    GridIndices2D indices = positionsToIndices(position);
    GridPositions2D grid_center_positions = indicesToPositions(indices);

    std::array<int,2> neighbor;

    for(int i = 0; i < 2; i++)
    {
        if((position[i] - grid_center_positions[i]) > 0)
        {
            neighbor[i] = 1;
        }
        else
        {
            neighbor[i] = -1;
        }
    }

    float l_x1 = fabs(position[0] - grid_center_positions[0]);
    float l_x2 = resolution_ - l_x1;
    float l_y1 = fabs(position[1] - grid_center_positions[1]);
    float l_y2 = resolution_ - l_y1;

    if(indices[0]+neighbor[0] < 0 || indices[0]+neighbor[0] >= dim_x_ || indices[1]+neighbor[1] < 0 || indices[1]+neighbor[1] >= dim_y_)
    {
        return 0;
    }

    ContactPoint pt1 = contact_point_list_[indices[0]][indices[1]];
    ContactPoint pt2 = contact_point_list_[indices[0]+neighbor[0]][indices[1]];
    ContactPoint pt3 = contact_point_list_[indices[0]][indices[1]+neighbor[1]];
    ContactPoint pt4 = contact_point_list_[indices[0]+neighbor[0]][indices[1]+neighbor[1]];

    float pt1_score, pt2_score, pt3_score, pt4_score;

    if(pt1.feasible_ && pt2.feasible_ && pt3.feasible_ && pt4.feasible_)
    {
        if(strcmp(feature_type.c_str(),"clearance"))
        {
            pt1_score = pt1.getClearanceScore(contact_type);
            pt2_score = pt2.getClearanceScore(contact_type);
            pt3_score = pt3.getClearanceScore(contact_type);
            pt4_score = pt4.getClearanceScore(contact_type);
        }
        else if(strcmp(feature_type.c_str(),"orientation"))
        {
            pt1_score = pt1.getOrientationScore(contact_direction);
            pt2_score = pt2.getOrientationScore(contact_direction);
            pt3_score = pt3.getOrientationScore(contact_direction);
            pt4_score = pt4.getOrientationScore(contact_direction);
        }
        else if(strcmp(feature_type.c_str(),"total"))
        {
            pt1_score = pt1.getTotalScore(contact_type,contact_direction);
            pt2_score = pt2.getTotalScore(contact_type,contact_direction);
            pt3_score = pt3.getTotalScore(contact_type,contact_direction);
            pt4_score = pt4.getTotalScore(contact_type,contact_direction);
        }

        Eigen::Matrix2f score_matrix;
        score_matrix(0,0) = pt1_score;
        score_matrix(1,0) = pt2_score;
        score_matrix(0,1) = pt3_score;
        score_matrix(1,1) = pt4_score;

        Eigen::MatrixXf M1(1,2);
        M1(0,0) = l_x2;
        M1(0,1) = l_x1;

        Eigen::MatrixXf M2(2,1);
        M2(0,0) = l_y2;
        M2(1,0) = l_y1;

        float interpolated_score = (1.0/((l_x1+l_x2)*(l_y1+l_y2))) * ((M1*score_matrix*M2)(0,0));

        return interpolated_score;
    }
    else
    {
        return 0;
    }

}