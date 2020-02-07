from __future__ import print_function

import numpy as np
import sys
import math
import heapq
import copy
import time
import pickle
import IPython

import openravepy as rave
import sklearn

from config_parameter import *
from transformation_conversion import *
from structures import structure, trimesh_surface
from geometry_mesh import *
from contact_region import *


class map_cell_2D: # x, y
    def __init__(self,x,y,ix,iy,height,foot_ground_projection,all_ground_structures): # the input x, and y are real x and y
        self.x = x
        self.y = y
        self.ix = ix
        self.iy = iy
        self.height = height
        self.foot_ground_projection = foot_ground_projection
        self.all_ground_structures = all_ground_structures

class map_cell_3D(map_cell_2D): # x, y, theta
    def __init__(self,parent,g,itheta,cell_2D,map_grid,left_hand_checking_surface_index,right_hand_checking_surface_index):

        self.parent = parent
        self.g = g # stores the heuristics, 9999.0: body in collision, 4999.0: infeasible, 999.0: before running Dijkstra
        self.h = 0
        self.step_num = 0

        self.theta = map_grid.min_theta + map_grid.angle_resolution * itheta
        self.itheta = itheta

        self.left_hand_checking_surface_index = left_hand_checking_surface_index
        self.right_hand_checking_surface_index = right_hand_checking_surface_index

        self.left_hand_contact_region_exist = None # None means unknown
        self.right_hand_contact_region_exist = None

        self.env_transition_feature = [0] * 12

        self.near_obstacle = False

        self.hand_contact_env_feature = None

        self.open = True

        map_cell_2D.__init__(self,cell_2D.x,cell_2D.y,cell_2D.ix,cell_2D.iy,cell_2D.height,cell_2D.foot_ground_projection,cell_2D.all_ground_structures)

    def get_f(self):
        return self.g + self.h

    def get_indices(self):
        return (self.ix,self.iy,self.itheta)

    def get_position(self):
        return (self.x,self.y,self.theta)


class map_cell_3D_motion_mode_augmented(map_cell_3D): # x, y, theta, L
    def __init__(self,parent,g,cell_3D,cell_2D,map_grid,motion_mode):
        map_cell_3D.__init__(self,parent,g,cell_3D.itheta,cell_2D,map_grid,cell_3D.left_hand_checking_surface_index,cell_3D.right_hand_checking_surface_index)

        self.motion_mode = motion_mode


class map_grid:
    def __init__(self,resolution,structures,or_robot,environment_boundaries=None):
        self.resolution = resolution
        self.angle_resolution = 30

        if environment_boundaries is None:
            # automatically decide the environment boundary
            # this assumes every structures is a surface
            env_min_x = sys.maxint
            env_max_x = -sys.maxint
            env_min_y = sys.maxint
            env_max_y = -sys.maxint

            for struct in structures:
                for vertex in struct.vertices:
                    env_min_x = min(vertex[0],env_min_x)
                    env_max_x = max(vertex[0],env_max_x)
                    env_min_y = min(vertex[1],env_min_y)
                    env_max_y = max(vertex[1],env_max_y)

        else:
            env_min_x = environment_boundaries[0]
            env_max_x = environment_boundaries[1]
            env_min_y = environment_boundaries[2]
            env_max_y = environment_boundaries[3]

        self.min_x = math.floor(env_min_x/resolution)*resolution - resolution/2.0
        self.max_x = math.ceil(env_max_x/resolution)*resolution + resolution/2.0
        self.min_y = math.floor(env_min_y/resolution)*resolution - resolution/2.0
        self.max_y = math.ceil(env_max_y/resolution)*resolution + resolution/2.0
        self.min_theta = -180
        self.max_theta = 150

        self.dim_x = int(math.ceil(abs(env_max_x)/resolution)) + int(math.ceil(abs(env_min_x)/resolution)) + 1
        self.dim_y = int(math.ceil(abs(env_max_y)/resolution)) + int(math.ceil(abs(env_min_y)/resolution)) + 1
        self.dim_theta = (self.max_theta - self.min_theta) / self.angle_resolution + 1

        self.left_foot_neighbor_window = {}
        self.right_foot_neighbor_window = {}
        self.chest_neighbor_window = {}
        self.max_step_size = 0

        self.cell_2D_list = [[None for j in xrange(self.dim_y)] for i in xrange(self.dim_x)]
        self.cell_3D_list = [[[None for k in xrange(self.dim_theta)] for j in xrange(self.dim_y)] for i in xrange(self.dim_x)]

        self.feet_cp_grid = None
        self.env_transition_feature_dict = {}
        self.env_transition_prediction_dict = {}
        self.env_transition_motion_mode_dict = {}
        self.env_transition_query_dict = {}

        self.env_transition_all_motion_mode_prediction_dict = {}
        self.env_transition_feature_vector_list_dict = {}

        # construct cell_2D_list and cell_3D_list
        self.heuristics_ground_mapping(structures,or_robot)

        self.footstep_transition_traversability_legs_only = None
        self.footstep_transition_traversability = None
        self.hand_transition_traversability = None

        # load the regressor
        try:
            print('Load traversability_regressor_two_hands...', end='')
            out_file = open(planning_data_path+'traversability_regressor_full_manipulators','r')
            self.traversability_regressor_two_hands = pickle.load(out_file)
            out_file.close()
            print('Done.')
        except Exception:
            raw_input('Fail.')
            return

        try:
            print('Load traversability_regressor_legs_and_one_hand...', end='')
            out_file = open(planning_data_path+'traversability_regressor_legs_and_one_hand','r')
            self.traversability_regressor_one_hand = pickle.load(out_file)
            out_file.close()
            print('Done.')
        except Exception:
            raw_input('Fail.')
            return

        try:
            print('Load traversability_regressor_legs_only...', end='')
            out_file = open(planning_data_path+'traversability_regressor_legs_only','r')
            self.traversability_regressor_legs_only = pickle.load(out_file)
            out_file.close()
            print('Done.')
        except Exception:
            raw_input('Fail.')
            return

        try:
            print('Load goal_index_dict...', end='')
            out_file = open(planning_data_path+'goal_index_dict','r')
            self.goal_index_dict = pickle.load(out_file)
            out_file.close()
            print('Done.')
        except Exception:
            raw_input('Fail.')
            return


    def get_all_torso_transition(self):

        torso_transition_set = set()

        for ix in range(self.dim_x):
            for iy in range(self.dim_y):
                for itheta in range(self.dim_theta):

                    if self.cell_3D_list[ix][iy][itheta].g != 9999.0 and self.cell_3D_list[ix][iy][itheta].g != 4999.0:
                        theta = self.grid_to_position_theta(itheta)

                        for nipos in self.chest_neighbor_window[theta]:
                            nix = nipos[0] + ix
                            niy = nipos[1] + iy

                            (nx,ny) = self.grid_to_position_xy((nix,niy))

                            if nix >= 0 and nix < self.dim_x and niy >= 0 and niy < self.dim_y:

                                for nitheta_t in range(itheta-1,itheta+2):
                                    nitheta = nitheta_t % self.dim_theta

                                    if self.cell_3D_list[nix][niy][nitheta].g != 9999.0 and self.cell_3D_list[nix][niy][nitheta].g != 4999.0:
                                        if ix != nix or iy != niy or itheta != nitheta:
                                            torso_transition_set.add((ix,iy,itheta,nix,niy))

        torso_transition_list = list(torso_transition_set)

        return torso_transition_list


    def get_transition_traversability_feature_vector(self,from_cell_index,to_cell_index,feature_type='clearance_only',motion_type='all_manipulators'):
        ix1 = from_cell_index[0]
        iy1 = from_cell_index[1]
        itheta1 = from_cell_index[2]

        ix2 = to_cell_index[0]
        iy2 = to_cell_index[1]
        itheta2 = to_cell_index[2]

        if (self.cell_3D_list[ix1][iy1][itheta1].g != 9999.0 and self.cell_3D_list[ix1][iy1][itheta1].g != 4999.0 and
            self.cell_3D_list[ix2][iy2][itheta2].g != 9999.0 and self.cell_3D_list[ix2][iy2][itheta2].g != 4999.0):

            if motion_type == 'legs_only':
                footstep_traversability = self.footstep_transition_traversability_legs_only[(ix1,iy1,itheta1,ix2,iy2)] # the return is a tuple of (clearance_score,oreintation_score,total_score)

                if feature_type == 'clearance_only':
                    return [footstep_traversability[0]]
                elif feature_type == 'combination':
                    return [footstep_traversability[0],footstep_traversability[1]]
                elif feature_type == 'total':
                    return [footstep_traversability[2]]
            else:
                hand_traversability = self.hand_transition_traversability[(ix1,iy1,itheta1)] # the return is a tuple of (clearance_score(4),oreintation_score(4),total_score(4))
                footstep_traversability = self.footstep_transition_traversability[(ix1,iy1,itheta1,ix2,iy2)] # the return is a tuple of (clearance_score,oreintation_score,total_score)

                if feature_type == 'clearance_only':
                    return [footstep_traversability[0],hand_traversability[0],hand_traversability[1],hand_traversability[2],hand_traversability[3]]
                elif feature_type == 'combination':
                    return [footstep_traversability[0],hand_traversability[0],hand_traversability[1],hand_traversability[2],hand_traversability[3],
                            footstep_traversability[1],hand_traversability[4],hand_traversability[5],hand_traversability[6],hand_traversability[7]]
                elif feature_type == 'total':
                    return [footstep_traversability[2],hand_traversability[8],hand_traversability[9],hand_traversability[10],hand_traversability[11]]

        else:
            return None


    def update_traversability_features(self,footstep_transition_traversability_legs_only,footstep_transition_traversability,hand_transition_traversability):
        self.hand_transition_traversability = hand_transition_traversability
        self.footstep_transition_traversability = footstep_transition_traversability
        self.footstep_transition_traversability_legs_only = footstep_transition_traversability_legs_only


    def get_env_transition_prediction(self,from_cell_index,to_cell_index,possible_motion_mode_index_list=[0,1,2,3]):
        (ix1,iy1,itheta1) = from_cell_index
        (ix2,iy2,itheta2) = to_cell_index

        feature_type = 'clearance_only'

        if (self.cell_3D_list[ix1][iy1][itheta1].g != 9999.0 and self.cell_3D_list[ix1][iy1][itheta1].g != 4999.0 and
            self.cell_3D_list[ix2][iy2][itheta2].g != 9999.0 and self.cell_3D_list[ix2][iy2][itheta2].g != 4999.0 and
            (ix1 != ix2 or iy1 != iy2)):

            if self.env_transition_prediction_dict.get((ix1,iy1,itheta1,ix2,iy2)) is None:

                total_feature_vector = self.get_transition_traversability_feature_vector(from_cell_index,to_cell_index,feature_type,'others')
                total_feature_vector_legs_only = self.get_transition_traversability_feature_vector(from_cell_index,to_cell_index,feature_type,'legs_only')

                if feature_type == 'clearance_only' or feature_type == 'total':
                    feature_vector_list = [copy.deepcopy(total_feature_vector),
                                           [total_feature_vector[0],total_feature_vector[1],total_feature_vector[2]],
                                           [total_feature_vector[0],total_feature_vector[3],total_feature_vector[4]],
                                           copy.deepcopy(total_feature_vector_legs_only)]
                elif feature_type == 'combination':
                    feature_vector_list = [copy.deepcopy(total_feature_vector),
                                           [total_feature_vector[0],total_feature_vector[1],total_feature_vector[2],total_feature_vector[5],total_feature_vector[6],total_feature_vector[7]],
                                           [total_feature_vector[0],total_feature_vector[3],total_feature_vector[4],total_feature_vector[5],total_feature_vector[8],total_feature_vector[9]],
                                           copy.deepcopy(total_feature_vector_legs_only)]


                theta1 = self.grid_to_position_theta(itheta1)

                window_theta = theta1 % 90

                if theta1 >= 0 and theta1 < 90:
                    window_dx = (ix2-ix1) * self.resolution
                    window_dy = (iy2-iy1) * self.resolution
                elif theta1 >= 90 and theta1 < 180:
                    window_dx = (iy2-iy1) * self.resolution
                    window_dy = -(ix2-ix1) * self.resolution
                elif theta1 >= -180 and theta1 < -90:
                    window_dx = -(ix2-ix1) * self.resolution
                    window_dy = -(iy2-iy1) * self.resolution
                elif theta1 >= -90 and theta1 < 0:
                    window_dx = -(iy2-iy1) * self.resolution
                    window_dy = (ix2-ix1) * self.resolution

                goal_index = self.goal_index_dict[(window_dx,window_dy,window_theta)]

                max_env_transition_num = -sys.maxint
                all_motion_mode_env_transition_num = [0,0,0,0]

                traversability_regressor_list = [self.traversability_regressor_two_hands[goal_index], # legs and two hands
                                                 self.traversability_regressor_one_hand[goal_index], # legs and left hand
                                                 self.traversability_regressor_one_hand[goal_index], # legs and right hand
                                                 self.traversability_regressor_legs_only[goal_index]] # legs only

                with sklearn.config_context(assume_finite=True):

                    for motion_mode_index,regressor_tuple in enumerate(traversability_regressor_list):

                        if motion_mode_index not in possible_motion_mode_index_list:
                            continue

                        feature_vector = feature_vector_list[motion_mode_index]
                        feature_dim = len(feature_vector)

                        if regressor_tuple is not None:
                            regressor = regressor_tuple[0]
                            feature_min_X = regressor_tuple[1]
                            feature_max_X = regressor_tuple[2]
                            feature_min_y = regressor_tuple[3]
                            feature_max_y = regressor_tuple[4]

                            normalized_feature_vector = [0] * feature_dim

                            for j in range(feature_dim):
                                normalized_feature_vector[j] = round(min(max(float(feature_vector[j]-feature_min_X[j])/float(feature_max_X[j]-feature_min_X[j]),0.0),1.0),2)

                            if feature_vector[0] != 0:
                                normalized_feature_tuple = tuple(normalized_feature_vector + [goal_index])
                                query_previous_prediction = self.env_transition_query_dict.get(normalized_feature_tuple)

                                if query_previous_prediction is None:
                                    normalized_env_transition_num = regressor.predict([normalized_feature_vector])[0]
                                    self.env_transition_query_dict[normalized_feature_tuple] = normalized_env_transition_num
                                else:
                                    normalized_env_transition_num = query_previous_prediction
                            else:
                                normalized_env_transition_num = 0

                            normalized_env_transition_num = min(max(normalized_env_transition_num,0.0),1.0)

                            env_transition_num = feature_min_y + 1.0 * normalized_env_transition_num * (feature_max_y-feature_min_y)

                            all_motion_mode_env_transition_num[motion_mode_index] = env_transition_num

                            if env_transition_num > max_env_transition_num:
                                max_env_transition_num = env_transition_num
                                self.env_transition_motion_mode_dict[(ix1,iy1,itheta1,ix2,iy2)] = motion_mode_index

                self.env_transition_prediction_dict[(ix1,iy1,itheta1,ix2,iy2)] = max_env_transition_num
                self.env_transition_all_motion_mode_prediction_dict[(ix1,iy1,itheta1,ix2,iy2)] = all_motion_mode_env_transition_num
                self.env_transition_feature_vector_list_dict[(ix1,iy1,itheta1,ix2,iy2)] = feature_vector_list

                if max_env_transition_num == 0:
                    self.env_transition_motion_mode_dict[(ix1,iy1,itheta1,ix2,iy2)] = 0

            return self.env_transition_prediction_dict[(ix1,iy1,itheta1,ix2,iy2)]

        else:
            rave.raveLogError('Wrong traversability query: (%i,%i,%i) --> (%i,%i,%i).',ix1,iy1,itheta1,ix2,iy2,itheta2)
            return 0



    def heuristics_ground_mapping(self,structures,or_robot):

        hgm_start = time.time()
        env = or_robot.env

        # Find out grid cells in which the foot will collide with high obstacles
        obs_structures = []
        handle_structures = []
        ground_structures = []

        for struct in structures:

            if struct.type == 'ground':
                ground_structures.append(struct)
            else:
                obs_structures.append(struct)

            if struct.type == 'others':
                handle_structures.append(struct)

        max_arm_reach_cylinder_radius = or_robot.max_arm_length + or_robot.shoulder_w/2.0
        max_body_collision_cylinder_radius = math.hypot(0.95 * or_robot.foot_h, or_robot.shoulder_w/2.0+0.2)

        max_armreach_cylinder = openrave_cylinder(env,'max_armreach_cylinder',(or_robot.shoulder_z-0.3,or_robot.shoulder_z+0.3),max_arm_reach_cylinder_radius)
        max_body_collision_cylinder = openrave_cylinder(env,'max_body_collision_cylinder',(0.5, or_robot.top_z), max_body_collision_cylinder_radius)


        left_armreach_fan = trimesh_fan(env,'left_armreach_fan',(or_robot.shoulder_z-0.3, or_robot.shoulder_z+0.3),
                                                                (90-or_robot.max_arm_forward_angle+10, 90+or_robot.max_arm_backward_angle-10),
                                                                or_robot.max_arm_length,
                                                                (0, or_robot.shoulder_w/2.0))

        right_armreach_fan = trimesh_fan(env,'right_armreach_fan',(or_robot.shoulder_z-0.3,or_robot.shoulder_z+0.3),
                                                                  (-90-or_robot.max_arm_backward_angle+10,-90+or_robot.max_arm_forward_angle-10),
                                                                  or_robot.max_arm_length,
                                                                  (0,-or_robot.shoulder_w/2.0))

        body_collision_box = or_robot.body_collision_box

        env.AddKinBody(left_armreach_fan)
        env.AddKinBody(right_armreach_fan)
        env.AddKinBody(max_armreach_cylinder)
        env.AddKinBody(max_body_collision_cylinder)

        initialize_kinbody = time.time()

        chest_transition_long_radius = or_robot.foot_transition_long_radius/2.0
        chest_transition_short_radius = or_robot.foot_transition_short_radius/2.0

        self.max_step_size = chest_transition_long_radius

        self.left_foot_neighbor_window = {}
        self.right_foot_neighbor_window = {}
        self.chest_neighbor_window = {}
        search_window_size = int(math.ceil(or_robot.foot_transition_long_radius / self.resolution))

        ######################################################
        for it in range(self.dim_theta):
            theta = self.grid_to_position_theta(it)
            inverse_ellipse_frame = np.array([[math.cos(theta*deg_to_rad),math.sin(theta*deg_to_rad)],[-math.sin(theta*deg_to_rad),math.cos(theta*deg_to_rad)]])

            temp_left_foot_neighbor_window = []
            temp_right_foot_neighbor_window = []
            temp_chest_neighbor_window = []

            for ix in range(-search_window_size,search_window_size+1):
                for iy in range(-search_window_size,search_window_size+1):
                    grid_position = np.array([[ix*self.resolution],[iy*self.resolution]])
                    transformed_grid_position = np.dot(inverse_ellipse_frame,grid_position)

                    c = (transformed_grid_position[0,0]/or_robot.foot_transition_long_radius)**2 + \
                        (transformed_grid_position[1,0]/or_robot.foot_transition_short_radius)**2

                    if c <= 1:
                        if iy > 0:
                            temp_left_foot_neighbor_window.append((ix,iy))
                        elif iy < 0:
                            temp_right_foot_neighbor_window.append((ix,iy))

                        if c <= 0.25:
                            temp_chest_neighbor_window.append((ix,iy))


            self.left_foot_neighbor_window[theta] = temp_left_foot_neighbor_window
            self.right_foot_neighbor_window[theta] = temp_right_foot_neighbor_window
            self.chest_neighbor_window[theta] = temp_chest_neighbor_window
        ##########################################################

        construct_transition_ellipse = time.time()

        # print('construct transition ellipse: %5.5f'%(construct_transition_ellipse-initialize_kinbody))

        ground_projection_ray = np.array([[0],[0],[-1]],dtype=float)

        # Find grids with a floor on its projection.
        for ix in range(self.dim_x):
            for iy in range(self.dim_y):

                (x,y) = self.grid_to_position_xy((ix,iy))

                temp_height = -99.0
                temp_foot_ground_projection = (None,None)
                temp_all_ground_structures = []

                marching_foot_point = np.array([[x],[y],[99.0],[1]],dtype=float)

                proj_structure = None
                free_radius = math.sqrt(2.0) * (self.resolution/2.0) + or_robot.foot_radius
                minimum_radius = or_robot.foot_radius

                for struct in ground_structures:
                    if math.hypot(x-struct.xo,y-struct.yo) < struct.circumscribed_radius:
                        projected_foot_point = struct.fast_projection_global_frame(marching_foot_point[0:3,0:1],ground_projection_ray)
                        if projected_foot_point is not None:
                            temp_all_ground_structures.append(struct.id)

                        if projected_foot_point is not None and temp_height < projected_foot_point[2,0] and struct.inside_polygon(projected_foot_point):
                            temp_height = projected_foot_point[2,0]
                            marching_foot_point[2,0] = projected_foot_point[2,0]
                            proj_structure = struct

                if proj_structure is not None:
                    if proj_structure.dist_to_boundary(marching_foot_point[0:3,0:1],free_radius,True):
                        temp_foot_ground_projection = (True,proj_structure.id) # With a safe projection surface.
                    else:
                        temp_foot_ground_projection = (False,proj_structure.id) # With a not safe projection surface.
                else:
                    temp_foot_ground_projection = (False,None) # No projection, empty gap.


                # construct the cell_2D object
                self.cell_2D_list[ix][iy] = map_cell_2D(x,y,ix,iy,temp_height,copy.deepcopy(temp_foot_ground_projection),copy.copy(temp_all_ground_structures))


        collision_box_checking_time = 0.0
        arm_reach_checking_time = 0.0

        # Find poses
        for ix in range(self.dim_x):
            for iy in range(self.dim_y):

                filtered_handle_structures = []
                filtered_handle_structures_constructed = False

                filtered_body_collision_structures = []
                filtered_body_collision_structures_constructed = False

                (x,y) = self.grid_to_position_xy((ix,iy))

                for it in range(self.dim_theta):
                    (x,y,theta) = self.grid_to_position((ix,iy,it))
                    opposite_it = ((((theta+180)+180) % 360 - 180) - self.min_theta) / self.angle_resolution
                    chest_transform = xyzrpy_to_SE3([x,y,self.cell_2D_list[ix][iy].height,0.0,0.0,theta])

                    left_standing_place_exist = False
                    right_standing_place_exist = False
                    body_in_collision = False

                    temp_data = -999.0

                    # simplified version of checking if there is place to stand in neighborhood, should consider the height difference and foot size, etc.
                    if self.cell_2D_list[ix][iy].height == -99.0:
                        for neighbor in self.left_foot_neighbor_window[theta]:
                            nix = ix+neighbor[0]
                            niy = iy+neighbor[1]


                            if nix >= 0 and nix < self.dim_x and niy >= 0 and niy < self.dim_y:
                                if self.cell_2D_list[nix][niy].height != -99.0:
                                    left_standing_place_exist = True
                                    break

                        if left_standing_place_exist:
                            for neighbor in self.right_foot_neighbor_window[theta]:
                                nix = ix+neighbor[0]
                                niy = iy+neighbor[1]


                                if nix >= 0 and nix < self.dim_x and niy >= 0 and niy < self.dim_y:
                                    if self.cell_2D_list[nix][niy].height != -99.0:
                                        right_standing_place_exist = True
                                        break

                        if not left_standing_place_exist or not right_standing_place_exist:
                            temp_data = 4999.0
                            self.cell_3D_list[ix][iy][it] = map_cell_3D(None,temp_data,it,self.cell_2D_list[ix][iy],self,[],[])
                            self.cell_3D_list[ix][iy][opposite_it] = map_cell_3D(None,temp_data,opposite_it,self.cell_2D_list[ix][iy],self,[],[])
                            continue

                    # check if there is collision with the collision box body. if not in collision and the env is under disturbance, check if the arm can reach handle
                    if not filtered_body_collision_structures_constructed:
                        before_checking = time.time()
                        max_body_collision_cylinder.SetTransform(np.array([[1,0,0,x],[0,1,0,y],[0,0,1,self.cell_2D_list[ix][iy].height],[0,0,0,1]],dtype=float))
                        for struct in obs_structures:
                            if math.hypot(x-struct.xo,y-struct.yo) < max_body_collision_cylinder_radius+struct.circumscribed_radius:
                                if env.CheckCollision(struct.kinbody,max_body_collision_cylinder):
                                    filtered_body_collision_structures.append(struct)

                        filtered_body_collision_structures_constructed = True
                        collision_box_checking_time = collision_box_checking_time + time.time() - before_checking


                    if len(filtered_body_collision_structures) != 0:
                        # utilize the symmetry to cut the checking iterations half.
                        body_collision_box.SetTransform(chest_transform)

                        for struct in filtered_body_collision_structures:
                            if env.CheckCollision(struct.kinbody,body_collision_box):
                                # print('In Collision.')
                                body_in_collision = True
                                break

                        if body_in_collision:
                            temp_data = 9999.0
                            self.cell_3D_list[ix][iy][it] = map_cell_3D(None,temp_data,it,self.cell_2D_list[ix][iy],self,[],[])
                            self.cell_3D_list[ix][iy][opposite_it] = map_cell_3D(None,temp_data,opposite_it,self.cell_2D_list[ix][iy],self,[],[])
                            continue

                    # check if there is any handle in reach
                    left_hand_contact_surface_index = []
                    right_hand_contact_surface_index = []

                    start_checking_arm_reach = time.time()

                    if not filtered_handle_structures_constructed:
                        max_armreach_cylinder.SetTransform(np.array([[1,0,0,x],[0,1,0,y],[0,0,1,self.cell_2D_list[ix][iy].height],[0,0,0,1]],dtype=float))
                        for struct in handle_structures:
                            if math.hypot(x-struct.xo,y-struct.yo) < max_body_collision_cylinder_radius+struct.circumscribed_radius:
                                if env.CheckCollision(struct.kinbody,max_armreach_cylinder):
                                    filtered_handle_structures.append(struct)

                        filtered_handle_structures_constructed = True

                    if temp_data == 9999.0:
                        raw_input('ERROR: arm reach checking BUG!!!')

                    if len(filtered_handle_structures) != 0:

                        left_hand_contact_in_range = False
                        right_hand_contact_in_range = False

                        left_armreach_fan.SetTransform(chest_transform)
                        right_armreach_fan.SetTransform(chest_transform)

                        for struct in filtered_handle_structures:
                            if env.CheckCollision(struct.kinbody,left_armreach_fan):
                                left_hand_contact_in_range = True
                                left_hand_contact_surface_index.append(struct.id)

                            if env.CheckCollision(struct.kinbody,right_armreach_fan):
                                right_hand_contact_in_range = True
                                right_hand_contact_surface_index.append(struct.id)

                        arm_reach_checking_time = arm_reach_checking_time + time.time() - start_checking_arm_reach


                    temp_data = 999.0
                    self.cell_3D_list[ix][iy][it] = map_cell_3D(None,temp_data,it,self.cell_2D_list[ix][iy],self,copy.deepcopy(left_hand_contact_surface_index),copy.deepcopy(right_hand_contact_surface_index))
                    self.cell_3D_list[ix][iy][opposite_it] = map_cell_3D(None,temp_data,opposite_it,self.cell_2D_list[ix][iy],self,copy.deepcopy(right_hand_contact_surface_index),copy.deepcopy(left_hand_contact_surface_index))

        env.Remove(left_armreach_fan)
        env.Remove(right_armreach_fan)
        env.Remove(max_armreach_cylinder)
        env.Remove(max_body_collision_cylinder)

        print('collision box checking time: %5.5f'%collision_box_checking_time)
        print('arm reach checking time: %5.5f'%arm_reach_checking_time)

        # construct near_obstacle_map to find out all the states which has neighbor states which will cause the robot to collide with the obstacles
        for ix in range(self.dim_x):
            for iy in range(self.dim_y):
                for itheta in range(self.dim_theta):
                    if self.cell_3D_list[ix][iy][itheta].g != 9999.0 and self.cell_3D_list[ix][iy][itheta].g != 4999.0:
                        neighbors = self.chest_neighbor_window[self.grid_to_position_theta(itheta)]
                        for n in neighbors:
                            nix = max(min(ix+n[0],self.dim_x-1),0)
                            niy = max(min(iy+n[1],self.dim_y-1),0)

                            neighbor_ithetas = [(itheta-1) % self.dim_theta, (itheta+1) % self.dim_theta]
                            for nitheta in neighbor_ithetas:
                                if self.cell_3D_list[nix][niy][nitheta].g == 9999.0:
                                    self.cell_3D_list[ix][iy][itheta].near_obstacle = True
                                    break
                    elif self.cell_3D_list[ix][iy][itheta].g == 9999.0:
                        self.cell_3D_list[ix][iy][itheta].near_obstacle = True


    def position_to_grid_xy(self,position):
        x = position[0]
        y = position[1]

        index_x = int(math.floor((x-self.min_x)/self.resolution))
        index_y = int(math.floor((y-self.min_y)/self.resolution))

        if index_x >= self.dim_x or index_x < 0 or index_y >= self.dim_y or index_y < 0:
            print('Error: Input position (%5.3f,%5.3f) out of bound.'%(x,y))
            raw_input()

        return (index_x,index_y)

    def position_to_grid_theta(self,position):
        if isinstance(position,int) or isinstance(position,float):
            theta = position
        elif len(position) == 3:
            theta = position[2]
        else:
            print('ERROR: Unexpected input position size in grid_to_position_theta, position: ')
            raw_input(position)
            theta = None

        index_theta = int(round(float((int(theta)+180) % 360 - 180 - self.min_theta) / self.angle_resolution)) % self.dim_theta

        return index_theta

    def position_to_grid(self,position):
        (index_x,index_y) = self.position_to_grid_xy(position)
        index_theta = self.position_to_grid_theta(position)

        return (index_x,index_y,index_theta)

    def position_to_dist(self,position):
        (index_x,index_y,index_theta) = self.position_to_grid(position)
        return self.cell_3D_list[index_x][index_y][index_theta].g

    def position_to_validity(self,position):
        dist = self.position_to_dist(position)

        return (dist != 4999.0 and dist != 9999.0)

    def position_to_step_num(self,position):
        (index_x,index_y,index_theta) = self.position_to_grid(position)
        return self.cell_3D_list[index_x][index_y][index_theta].step_num

    def position_to_left_hand_checking_surface_indices(self,position):
        (index_x,index_y,index_theta) = self.position_to_grid(position)
        return self.cell_3D_list[index_x][index_y][index_theta].left_hand_checking_surface_index

    def position_to_right_hand_checking_surface_indices(self,position):
        (index_x,index_y,index_theta) = self.position_to_grid(position)
        return self.cell_3D_list[index_x][index_y][index_theta].right_hand_checking_surface_index

    def position_to_combined_hand_checking_surface_indices(self,position):
        return list(set(self.position_to_left_hand_checking_surface_indices(position)+self.position_to_right_hand_checking_surface_indices(position)))

    def grid_to_dist(self,grid):
        return self.cell_3D_list[grid[0]][grid[1]][grid[2]].g

    def grid_to_validity(self,grid):
        dist = self.grid_to_dist(grid)

        return (dist != 4999.0 and dist != 9999.0)

    def grid_to_step_num(self,grid):
        return self.cell_3D_list[grid[0]][grid[1]][grid[2]].step_num

    def grid_to_left_hand_checking_surface_indices(self,grid):
        return self.cell_3D_list[grid[0]][grid[1]][grid[2]].left_hand_checking_surface_index

    def grid_to_right_hand_checking_surface_indices(self,grid):
        return self.cell_3D_list[grid[0]][grid[1]][grid[2]].right_hand_checking_surface_index

    def grid_to_combined_hand_checking_surface_indices(self,grid):
        return list(set(self.cell_3D_list[grid[0]][grid[1]][grid[2]].left_hand_checking_surface_index+self.cell_3D_list[grid[0]][grid[1]][grid[2]].right_hand_checking_surface_index))

    def grid_path_to_position_path(self,grid_path):
        position_path = copy.deepcopy(grid_path)
        for i in range(len(grid_path)):
            position_path[i] = self.grid_to_position(grid_path[i])

        return position_path

    def grid_to_position_xy(self,indices):
        index_x = indices[0]
        index_y = indices[1]

        x = self.min_x + self.resolution * (index_x + 0.5)
        y = self.min_y + self.resolution * (index_y + 0.5)

        return (x,y)

    def grid_to_position_theta(self,indices):
        if isinstance(indices,int) or isinstance(indices,float):
            index_theta = indices
        elif len(indices) == 3:
            index_theta = indices[2]
        else:
            print('ERROR: Unexpected input indices size in grid_to_position_theta, indices: ')
            raw_input(indices)
            index_theta = None

        theta = self.min_theta + self.angle_resolution * index_theta

        return theta

    def grid_to_position(self,indices):

        (x,y) = self.grid_to_position_xy(indices)
        theta = self.grid_to_position_theta(indices)

        return (x,y,theta)

    def get_foot_ground_projection(self,position):
        (index_x,index_y) = self.position_to_grid_xy(position)

        return self.cell_2D_list[index_x][index_y].foot_ground_projection

    def get_neighboring_ground_structure_ids(self,position):
        (index_x,index_y) = self.position_to_grid_xy(position)

        all_ground_structures = []

        for i in range(-1,2,1):
            for j in range(-1,2,1):
                if index_x+i >= 0 and index_x+i < self.dim_x and index_y+j >= 0 and index_y+j < self.dim_y:
                    all_ground_structures = all_ground_structures + self.cell_2D_list[index_x][index_y].all_ground_structures

        all_ground_structures = list(set(all_ground_structures))

        return all_ground_structures

    def cell_inside_grid(self,cell_indices):
        return (cell_indices[0] >= 0 and cell_indices[0] < self.dim_x and
                cell_indices[1] >= 0 and cell_indices[1] < self.dim_y and
                cell_indices[2] >= 0 and cell_indices[2] < self.dim_theta)

    def cell_inside_grid_2d(self,cell_indices):
        return (cell_indices[0] >= 0 and cell_indices[0] < self.dim_x and
                cell_indices[1] >= 0 and cell_indices[1] < self.dim_y)

    def position_inside_grid(self,cell_position):
        return (cell_position[0] >= self.min_x and cell_position[0] < self.max_x and
                cell_position[1] >= self.min_y and cell_position[1] < self.max_y and
                cell_position[2] >= self.min_theta and cell_position[2] < self.max_theta)

    def position_inside_grid_2d(self,cell_position):
        return (cell_position[0] >= self.min_x and cell_position[0] < self.max_x and
                cell_position[1] >= self.min_y and cell_position[1] < self.max_y)


    def path_planning(self,goal,start=None,method='Dijkstra',direction='backward',exhaust_the_map=True,contact_bias=False,contact_regions=None,env_transition_bias=False,search_space=None,consider_motion_mode=False): # planning from goal to start
        start_time = time.time()

        rave.raveLogInfo('Start Map Grid Path Planning: Method: %s.'%(method))

        if consider_motion_mode and not env_transition_bias:
            rave.raveLogError('If consider_motion_mode is enabled, the env_transition_bias should be enabled, too.')
            raw_input()
            return

        if consider_motion_mode:
            self.cell_3D_mm_list = [[[[ map_cell_3D_motion_mode_augmented(None,
                                                                      self.cell_3D_list[i][j][k].g,
                                                                      self.cell_3D_list[i][j][k],
                                                                      self.cell_2D_list[i][j],self,l)
                                    for l in xrange(4)]
                                    for k in xrange(self.dim_theta)]
                                    for j in xrange(self.dim_y)]
                                    for i in xrange(self.dim_x)]


        cell_transition_query_counter = 0

        openHeap = []

        (gix,giy,githeta) = self.position_to_grid(goal)
        if start is not None:
            (stix,stiy,stitheta) = self.position_to_grid(start)

        if abs(self.cell_3D_list[gix][giy][githeta].g - 999.0) > 0.01:
            print('Goal grid cell infeasible: %5.1f'%(self.cell_3D_list[gix][giy][githeta].g))
            return False

        # set the distance of the goal to the goal as zero
        self.cell_3D_list[gix][giy][githeta].g = 0.0
        if consider_motion_mode:
            self.cell_3D_mm_list[gix][giy][githeta][0].g = 0.0

        if method == 'A_Star':
            if consider_motion_mode:
                self.cell_3D_mm_list[gix][giy][githeta][0].h = self.h_estimation((gix,giy,githeta),(stix,stiy,stitheta))
            else:
                self.cell_3D_list[gix][giy][githeta].h = self.h_estimation((gix,giy,githeta),(stix,stiy,stitheta))
        elif method == 'Dijkstra':
            self.cell_3D_list[gix][giy][githeta].h = 0

        if consider_motion_mode:
            self.cell_3D_mm_list[gix][giy][githeta][0].parent = None
            openHeap.append((self.cell_3D_mm_list[gix][giy][githeta][0].get_f(),(gix,giy,githeta,0)))
        else:
            self.cell_3D_list[gix][giy][githeta].parent = None
            openHeap.append((self.cell_3D_list[gix][giy][githeta].get_f(),(gix,giy,githeta)))

        heapq.heapify(openHeap)

        prev_time = time.time()

        first_time = True

        print('Preprocess Time: %5.3f.'%(prev_time-start_time))

        while first_time:

            first_time = False
            state_counter = 0

            while openHeap:

                if consider_motion_mode:
                    (f,(ix,iy,itheta,imm)) = heapq.heappop(openHeap)
                    current_f = self.cell_3D_mm_list[ix][iy][itheta][imm].get_f()
                    is_open = self.cell_3D_mm_list[ix][iy][itheta][imm].open
                else:
                    (f,(ix,iy,itheta)) = heapq.heappop(openHeap)
                    current_f = self.cell_3D_list[ix][iy][itheta].get_f()
                    is_open = self.cell_3D_list[ix][iy][itheta].open

                if (self.cell_3D_list[ix][iy][itheta].g != 9999.0 and
                    self.cell_3D_list[ix][iy][itheta].g != 4999.0 and
                    is_open and
                    (current_f >= f)):

                    state_counter = state_counter + 1

                    if consider_motion_mode:
                        self.cell_3D_mm_list[ix][iy][itheta][imm].open = False
                    else:
                        self.cell_3D_list[ix][iy][itheta].open = False


                    if not exhaust_the_map and start is not None and ix == stix and iy == stiy and itheta == stitheta:
                        path = []

                        if consider_motion_mode:
                            node = self.cell_3D_mm_list[ix][iy][itheta][imm]
                            while node is not None:
                                path.append(node)
                                node = node.parent

                            # copy the motion mode that reaches the final cell to the motion mode of last cell in the path
                            path[-1].motion_mode = path[-2].motion_mode

                            print('cell transition query: %d.'%(cell_transition_query_counter))
                            return path
                        else:
                            node = self.cell_3D_list[ix][iy][itheta]
                            while node is not None:
                                path.append(node)
                                node = node.parent

                            print('cell transition query: %d.'%(cell_transition_query_counter))
                            return path

                    theta = self.grid_to_position_theta(itheta)
                    theta_rad = theta * deg_to_rad

                    neighbors = self.chest_neighbor_window[theta]
                    for n in neighbors:

                        if ix+n[0] > self.dim_x-1 or ix+n[0] < 0:
                            continue
                        else:
                            nix = ix+n[0]

                        if iy+n[1] > self.dim_y-1 or iy+n[1] < 0:
                            continue
                        else:
                            niy = iy+n[1]

                        if nix != ix or niy != iy:
                            if (self.cell_3D_list[nix][niy][itheta].g != 9999.0 and
                                self.cell_3D_list[nix][niy][itheta].g != 4999.0):


                                next_itheta = [(itheta-1) % self.dim_theta, itheta, (itheta+1) % self.dim_theta]
                                for nitheta in next_itheta:
                                    ntheta = self.grid_to_position_theta(nitheta)
                                    ntheta_rad = ntheta * deg_to_rad
                                    next_neighbors = self.chest_neighbor_window[ntheta]

                                    if ((ix-nix,iy-niy) in next_neighbors and
                                         self.cell_3D_list[nix][niy][nitheta].g != 9999.0 and
                                         self.cell_3D_list[nix][niy][nitheta].g != 4999.0 and
                                         (search_space is None or (nix,niy,nitheta) in search_space) and
                                         self.cell_3D_list[nix][niy][nitheta].open):

                                        collision = False

                                        if self.cell_3D_list[ix][iy][itheta].near_obstacle:

                                            if nitheta != itheta:
                                                check_itheta = [itheta,nitheta]
                                            else:
                                                check_itheta = [itheta]

                                            for citheta in check_itheta:
                                                if abs(nix-ix) > abs(niy-iy):
                                                    six = min(nix,ix)
                                                    bix = max(nix,ix)
                                                    if six == ix:
                                                        siy = iy
                                                        biy = niy
                                                    else:
                                                        siy = niy
                                                        biy = iy

                                                    m = (biy-siy)/(bix-six)
                                                    for iix in range(six,bix):
                                                        iiy = int(round(m*(iix-six) + siy))
                                                        if self.cell_3D_list[iix][iiy][citheta].g == 9999.0:
                                                            collision = True
                                                            break
                                                else:
                                                    siy = min(niy,iy)
                                                    biy = max(niy,iy)
                                                    if siy == iy:
                                                        six = ix
                                                        bix = nix
                                                    else:
                                                        six = nix
                                                        bix = ix

                                                    m = (bix-six)/(biy-siy)
                                                    for iiy in range(siy,biy):
                                                        iix = int(round(m*(iiy-siy) + six))
                                                        if self.cell_3D_list[iix][iiy][citheta].g == 9999.0:
                                                            collision = True
                                                            break

                                                if collision:
                                                    break

                                        if not collision:
                                            # encourage the robot to walk backward from goal.(walk forward in execution)
                                            if direction == 'backward':

                                                if consider_motion_mode:
                                                    cost_penalty = 1 + 0.25 * ((math.cos(theta_rad)*(nix-ix) + math.sin(theta_rad)*(niy-iy)) / math.hypot(nix-ix,niy-iy) - (-1))
                                                else:
                                                    cost_penalty = 1 + 0.25 * ((math.cos(theta_rad)*(nix-ix) + math.sin(theta_rad)*(niy-iy)) / math.hypot(nix-ix,niy-iy) - (-1))

                                            elif direction == 'forward':
                                                if math.cos(theta_rad)*(ix-nix) + math.sin(theta_rad)*(iy-niy) > 0.5 * math.hypot(nix-ix,niy-iy):
                                                    cost_penalty = 1.5
                                                else:
                                                    cost_penalty = 1.0

                                            env_transition_cost = 0
                                            if env_transition_bias:

                                                ###############CELL TRANSITION#################
                                                if ix != nix or iy != niy:

                                                    # filter out all possible motion mode based on the existence hand contact structures
                                                    left_hand_contact_exist = (self.cell_3D_list[ix][iy][itheta].left_hand_checking_surface_index and
                                                                                self.cell_3D_list[nix][niy][nitheta].left_hand_checking_surface_index)
                                                    right_hand_contact_exist = (self.cell_3D_list[ix][iy][itheta].right_hand_checking_surface_index and
                                                                                self.cell_3D_list[nix][niy][nitheta].right_hand_checking_surface_index)

                                                    if consider_motion_mode:
                                                        possible_motion_mode_index_list = []
                                                        if self.cell_3D_mm_list[nix][niy][nitheta][3].open:
                                                            possible_motion_mode_index_list.append(3)

                                                        if left_hand_contact_exist and self.cell_3D_mm_list[nix][niy][nitheta][1].open:
                                                            possible_motion_mode_index_list.append(1)

                                                        if right_hand_contact_exist and self.cell_3D_mm_list[nix][niy][nitheta][2].open:
                                                            possible_motion_mode_index_list.append(2)

                                                        if left_hand_contact_exist and right_hand_contact_exist and self.cell_3D_mm_list[nix][niy][nitheta][0].open:
                                                            possible_motion_mode_index_list.append(0)

                                                        if not possible_motion_mode_index_list:
                                                            continue
                                                    else:
                                                        possible_motion_mode_index_list = [3]

                                                        if left_hand_contact_exist:
                                                            possible_motion_mode_index_list.append(1)

                                                        if right_hand_contact_exist:
                                                            possible_motion_mode_index_list.append(2)

                                                        if left_hand_contact_exist and right_hand_contact_exist:
                                                            possible_motion_mode_index_list.append(0)

                                                    cell_transition_query_counter += 1

                                                    # env_transition_predict = self.env_transition_prediction_dict.get((nix,niy,nitheta,ix,iy,itheta))
                                                    env_transition_predict = self.get_env_transition_prediction((nix,niy,nitheta),(ix,iy,itheta),possible_motion_mode_index_list)

                                                    if env_transition_predict is None:
                                                        env_transition_predict = 0.0


                                                    env_transition_cost = math.exp(-env_transition_predict)
                                                ###############CELL TRANSITION#################

                                            if consider_motion_mode:
                                                mm_delta_g_dict = {}

                                                for motion_mode in possible_motion_mode_index_list:
                                                    env_transition_predict = self.env_transition_all_motion_mode_prediction_dict[(nix,niy,nitheta,ix,iy)][motion_mode]
                                                    env_transition_cost = math.exp(-env_transition_predict)

                                                    if motion_mode == imm or (ix == gix and iy == giy and itheta == githeta):
                                                        mmc_cost = 0
                                                    else:
                                                        mmc_cost = motion_mode_changing_cost_ratio

                                                    delta_g = (self.resolution * math.hypot(nix-ix,niy-iy) +
                                                                (nitheta!=itheta) * self.angle_resolution * dist_angle_cost_ratio +
                                                                dist_step_cost_ratio * 1 +
                                                                dist_env_transition_cost_ratio * env_transition_cost +
                                                                mmc_cost) * cost_penalty

                                                    mm_delta_g_dict[motion_mode] = delta_g

                                            else:
                                                delta_g = (self.resolution * math.hypot(nix-ix,niy-iy) +
                                                            (nitheta!=itheta) * self.angle_resolution * dist_angle_cost_ratio +
                                                            dist_step_cost_ratio * 1 +
                                                            dist_env_transition_cost_ratio * env_transition_cost) * cost_penalty


                                            if consider_motion_mode:

                                                for nmm, delta_g in mm_delta_g_dict.iteritems():
                                                    alt = self.cell_3D_mm_list[ix][iy][itheta][imm].g + delta_g
                                                    if alt < self.cell_3D_mm_list[nix][niy][nitheta][nmm].g:
                                                        self.cell_3D_mm_list[nix][niy][nitheta][nmm].g = alt

                                                        if method == 'A_Star':
                                                            self.cell_3D_mm_list[nix][niy][nitheta][nmm].h = self.h_estimation((nix,niy,nitheta),(stix,stiy,stitheta))
                                                        elif method == 'Dijkstra':
                                                            self.cell_3D_mm_list[nix][niy][nitheta][nmm].h = 0

                                                        self.cell_3D_mm_list[nix][niy][nitheta][nmm].parent = self.cell_3D_mm_list[ix][iy][itheta][imm]
                                                        self.cell_3D_mm_list[nix][niy][nitheta][nmm].step_num = self.cell_3D_mm_list[ix][iy][itheta][imm].step_num + 1
                                                        heapq.heappush(openHeap, (self.cell_3D_mm_list[nix][niy][nitheta][nmm].get_f(),(nix,niy,nitheta,nmm)))

                                            else:
                                                alt = self.cell_3D_list[ix][iy][itheta].g + delta_g
                                                if alt < self.cell_3D_list[nix][niy][nitheta].g:
                                                    self.cell_3D_list[nix][niy][nitheta].g = alt
                                                    if method == 'A_Star' and self.cell_3D_list[nix][niy][nitheta].h == 0:
                                                        self.cell_3D_list[nix][niy][nitheta].h = self.h_estimation((nix,niy,nitheta),(stix,stiy,stitheta))
                                                    elif method == 'Dijkstra':
                                                        self.cell_3D_list[nix][niy][nitheta].h = 0

                                                    self.cell_3D_list[nix][niy][nitheta].parent = self.cell_3D_list[ix][iy][itheta]
                                                    self.cell_3D_list[nix][niy][nitheta].step_num = self.cell_3D_list[ix][iy][itheta].step_num + 1
                                                    heapq.heappush(openHeap, (self.cell_3D_list[nix][niy][nitheta].get_f(),(nix,niy,nitheta)))


            # raw_input('Dijkstra Done. Time: %5.3f'%(time.time() - start_time))
            print('openHeap exhausted.')


        if exhaust_the_map:
            print('cell transition query: %d.'%(cell_transition_query_counter))
            return True
        else:
            return []


    def h_estimation(self,indices,target):
        dist = math.hypot(target[0]-indices[0],target[1]-indices[1])
        step_num = math.ceil(self.resolution * dist / self.max_step_size)
        angular_dist = min(abs(indices[2]-target[2]),self.dim_theta-abs(indices[2]-target[2]))

        return self.resolution * dist + dist_angle_cost_ratio * self.angle_resolution * angular_dist + dist_step_cost_ratio * step_num


    def get_cell_height(self,cell):
        return self.cell_2D_list[cell[0]][cell[1]].height


    def get_neighbor_cell_indices(self, torso_path, neighbor_distance=1.0):

        neighbor_cell_distance = int(math.ceil(neighbor_distance / self.resolution))

        # find all the cells on the torso path
        torso_path_cell_indices = set()

        for i,cell in enumerate(torso_path):
            if i != 0:
                last_cell = torso_path[i-1]
                (ix,iy,itheta) = last_cell.get_indices()
                (nix,niy,nitheta) = cell.get_indices()

                if abs(nix-ix) > abs(niy-iy):
                    if nix > ix:
                        ix_range = range(ix,nix+1)
                    else:
                        ix_range = range(ix,nix-1,-1)

                    for cx in ix_range:
                        cy = int((cx-ix)*(niy-iy)*1.0/(nix-ix)+iy)
                        torso_path_cell_indices.add((cx,cy,itheta))

                        if itheta != nitheta:
                            torso_path_cell_indices.add((cx,cy,nitheta))
                else:
                    if niy > iy:
                        iy_range = range(iy,niy+1)
                    else:
                        iy_range = range(iy,niy-1,-1)

                    for cy in iy_range:
                        cx = int((cy-iy)*(nix-ix)*1.0/(niy-iy)+ix)
                        torso_path_cell_indices.add((cx,cy,itheta))

                        if itheta != nitheta:
                            torso_path_cell_indices.add((cx,cy,nitheta))

        neighbor_cell_indices = set()

        for cell_indices in torso_path_cell_indices:
            (ix,iy,itheta) = cell_indices

            theta = self.grid_to_position_theta(itheta)
            theta_rad = theta * deg_to_rad

            max_ix = min(ix+neighbor_cell_distance,self.dim_x-1)
            min_ix = max(ix-neighbor_cell_distance,0)
            max_iy = min(iy+neighbor_cell_distance,self.dim_y-1)
            min_iy = max(iy-neighbor_cell_distance,0)
            max_itheta = itheta+3
            min_itheta = itheta-3

            for cx in range(min_ix,max_ix+1):
                for cy in range(min_iy,max_iy+1):
                    lx = math.cos(theta_rad) * (cx-ix) + math.sin(theta_rad) * (cy-iy) # project the displacement to the orientation axis
                    ly = -math.sin(theta_rad) * (cx-ix) + math.cos(theta_rad) * (cy-iy) # project the displacement to the orientation axis

                    if math.sqrt(lx**2 + ly**2) <= neighbor_cell_distance:
                        for ctheta in range(min_itheta,max_itheta+1):
                            neighbor_cell_indices.add((cx,cy,ctheta%self.dim_theta))

        return neighbor_cell_indices


    def reset_cells_cost(self):

        for ix in range(self.dim_x):
            for iy in range(self.dim_y):
                for itheta in range(self.dim_theta):
                    self.cell_3D_list[ix][iy][itheta].parent = None

                    if self.cell_3D_list[ix][iy][itheta].g != 4999.0 and self.cell_3D_list[ix][iy][itheta].g != 9999.0:
                        self.cell_3D_list[ix][iy][itheta].g = 999.0

                    self.cell_3D_list[ix][iy][itheta].h = 0
                    self.cell_3D_list[ix][iy][itheta].step_num = 0
                    self.cell_3D_list[ix][iy][itheta].open = True

