#include "Utilities.hpp"
// #include "PointGrid.hpp"

PointGrid::PointGrid(float _min_x, float _max_x, float _min_y, float _max_y, float _resolution)
{
    initializeParameters(_min_x, _max_x, _min_y, _max_y, _resolution);
}

void  PointGrid::initializeParameters(float _min_x, float _max_x, float _min_y, float _max_y, float _resolution)
{
    resolution_ = _resolution;

    dim_x_ = int((_max_x - _min_x)/_resolution);
    dim_y_ = int((_max_y - _min_y)/_resolution);

    min_x_ = _min_x + ((_max_x-_min_x) - (_resolution*dim_x_))/2.0;
    max_x_ = _max_x - ((_max_x-_min_x) - (_resolution*dim_x_))/2.0;
    min_y_ = _min_y + ((_max_y-_min_y) - (_resolution*dim_y_))/2.0;
    max_y_ = _max_y - ((_max_y-_min_y) - (_resolution*dim_y_))/2.0;

    center_x_ = min_x_ + int(dim_x_/2.0) * _resolution + _resolution/2.0;
    center_y_ = min_y_ + int(dim_y_/2.0) * _resolution + _resolution/2.0;

    // dim_x_ = int(round((max_x_ - min_x_)/_resolution));
    // dim_y_ = int(round((max_y_ - min_y_)/_resolution));
}

GridIndices2D PointGrid::positionsToIndices(GridPositions2D position)
{
    float x = position[0];
    float y = position[1];

    int index_x = int(floor((x-min_x_)/resolution_));
    int index_y = int(floor((y-min_y_)/resolution_));

    if(index_x >= dim_x_ || index_x < 0 || index_y >= dim_y_ ||  index_y < 0)
    {
        RAVELOG_ERROR("Error: Input position (%5.3f,%5.3f) out of bound.\n",x,y);
    }

    return {index_x,index_y};
}

GridPositions2D PointGrid::indicesToPositions(GridIndices2D indices)
{
    int index_x = indices[0];
    int index_y = indices[1];

    float position_x = min_x_ + (index_x+0.5) * resolution_;
    float position_y = min_y_ + (index_y+0.5) * resolution_;

    if(index_x >= dim_x_ || index_x < 0 || index_y >= dim_y_ ||  index_y < 0)
    {
        RAVELOG_ERROR("Error: Input index (%d,%d) out of bound: Dim: (%d,%d).\n",index_x,index_y,dim_x_,dim_y_);
    }

    return {position_x,position_y};
}

float PointGrid::getInterpolatedScore(std::array<float,4> scores, std::array<float,4> dist_to_boundaries)
{
    Eigen::Matrix2f score_matrix;
    score_matrix(0,0) = scores[0];
    score_matrix(1,0) = scores[1];
    score_matrix(0,1) = scores[2];
    score_matrix(1,1) = scores[3];

    float l_x1 = dist_to_boundaries[0];
    float l_x2 = dist_to_boundaries[1];
    float l_y1 = dist_to_boundaries[2];
    float l_y2 = dist_to_boundaries[3];

    Eigen::MatrixXf M1(1,2);
    M1(0,0) = l_x2;
    M1(0,1) = l_x1;

    Eigen::MatrixXf M2(2,1);
    M2(0,0) = l_y2;
    M2(1,0) = l_y1;    

    float interpolated_score = (1.0/((l_x1+l_x2)*(l_y1+l_y2))) * ((M1*score_matrix*M2)(0,0));

    return interpolated_score;
}
