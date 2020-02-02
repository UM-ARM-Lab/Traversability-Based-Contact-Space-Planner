#ifndef MAPGRID_HPP
#define MAPGRID_HPP

// #include "Utilities.hpp"

// // OpenRAVE
// #include <openrave/plugin.h>

class MapCell2D
{
public:
    MapCell2D(float _x, float _y, float _ix, float _iy):
    x_(_x),
    y_(_y),
    ix_(_ix),
    iy_(_iy)
    {};

    float x_;
    float y_;
    int ix_;
    int iy_;
    
    float height_;

    std::pair<bool, std::shared_ptr<TrimeshSurface> > foot_ground_projection_;
    std::vector< std::shared_ptr<TrimeshSurface> > all_ground_structures_;

};

class MapCell3D : MapCell2D
{
public:
    MapCell3D(float _x, float _y, float _theta, float _ix, float _iy, float _itheta):
    MapCell2D(_x, _y, _ix, _iy),
    theta_(_theta),
    itheta_(_itheta),
    parent_indices_({-99,-99,-99})
    {};

    std::array<int,3> parent_indices_;

    float g_;
    float h_;
    int step_num_;

    float theta_;
    int itheta_;

    std::vector< std::shared_ptr<TrimeshSurface> > left_hand_checking_surfaces_;
    std::vector< std::shared_ptr<TrimeshSurface> > right_hand_checking_surfaces_;

    bool left_hand_contact_region_exist_;
    bool right_hand_contact_region_exist_;

    bool near_obstacle_;

    std::array<float,4> hand_contact_env_feature_;

    inline float getF() {return (g_ + h_);}

    int getTravelDirection(MapCell3D goal_cell);

    inline GridIndices3D getIndices() {return {ix_, iy_, itheta_};}
    inline GridPositions3D getPositions() {return {x_, y_, theta_};}
};


class MapGrid
{
public:
    MapGrid(float _min_x, float _max_x, float _min_y, float _max_y, float _xy_resolution, float _theta_resolution);

    GridIndices2D positionsToIndicesXY(GridPositions2D xy_position);
    int positionsToIndicesTheta(float theta_position);
    GridIndices3D positionsToIndices(GridPositions3D position);

    GridPositions2D indicesToPositionsXY(GridIndices2D xy_indices);
    float indicesToPositionsTheta(int theta_index);
    GridPositions3D indicesToPositions(GridIndices3D indices);
    
    inline std::array<int,2> getXYDimensions(){return {dim_x_, dim_y_};}
    inline int getThetaDimensions(){return dim_theta_;}
    inline std::array<int,3> getDimensions(){return {dim_x_, dim_y_, dim_theta_};}

    inline std::array<float,6> getBoundaries(){return {min_x_, max_x_, min_y_, max_y_, min_theta_, max_theta_};}
    inline float getXYResolution(){return xy_resolution_;}
    inline bool insideGrid(GridPositions3D positions){return (positions[0] >= min_x_ && positions[0] < max_x_ && positions[1] >= min_y_ && positions[1] < max_y_ && positions[2] >= min_theta_ && positions[2] < max_theta_);}
    inline bool insideGrid(GridIndices3D indices){return (indices[0] >= 0 && indices[0] < dim_x_ && indices[1] >= 0 && indices[1] < dim_y_ && indices[2] >= 0 && indices[2] < dim_theta_);}

    const float xy_resolution_;
    const float theta_resolution_;
    
    const int dim_x_;
    const int dim_y_;
    const int dim_theta_;

    const float min_x_;
    const float max_x_;
    const float min_y_;
    const float max_y_;
    const float min_theta_;
    const float max_theta_;

    std::map<int, std::vector< GridIndices2D > > left_foot_neighbor_window_;
    std::map<int, std::vector< GridIndices2D > > right_foot_neighbor_window_;
    std::map<int, std::vector< GridIndices2D > > torso_neighbor_window_;

    int max_step_size;

    std::vector< std::vector<MapCell2D> > cell_2D_list_;
    std::vector< std::vector< std::vector<MapCell3D> > > cell_3D_list_;

    // self.feet_cp_grid = None

    std::map< std::array<int,5>, float > footstep_env_transition_feature_dict_;
    std::map< std::array<int,3>, std::array<float,4> > hand_env_transition_feature_dict_;
    // self.env_transition_feature_dict = {}
    // self.env_transition_prediction_dict = {}

};

#endif