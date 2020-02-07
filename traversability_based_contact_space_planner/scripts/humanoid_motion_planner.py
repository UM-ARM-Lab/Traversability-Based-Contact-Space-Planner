from __future__ import print_function
import numpy as np
import math
import openravepy as rave
import copy
import time
import pickle
import os
import IPython

# CPP Utility OpenRAVE Interface imports
from cbirrtpy import CBiRRT
from elasticstripspy import ElasticStrips

# robot loader imports
import load_escher

# local imports
from config_parameter import *
from transformation_conversion import *
from drawing_functions import *
from motion_plan_library_matching import plan_env_matching
from contact_space_planner import ANA_Star, h_estimation, node
from step_interpolation import step_interpolation
from environment_handler import environment_handler
from map_grid import map_grid
from traversability_cpp_utility_wrapper import traversability_cpp_utility_wrapper
from torso_path_segmentation import get_torso_path_segmentation, get_torso_path_segment_traversability_score_mean, get_torso_path_segment_traversability_score_max
from motion_plan import *
from contact_region import *
from contact_transition import *


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


class path_segment:
    def __init__(self,torso_path_boundary_indices,motion_mode):
        self.torso_path_boundary_indices = torso_path_boundary_indices
        self.path_generation_method = None
        self.contact_state_path = None
        self.traj = None
        self.waypoint_contact_manips = None
        self.motion_mode = motion_mode

    def update_path_segment(self, contact_state_path=None, traj=None, waypoint_contact_manips=None, path_generation_method=None):
        if contact_state_path is not None:
            self.contact_state_path = copy.deepcopy(contact_state_path)

        if traj is not None:
            self.traj = rave.RaveCreateTrajectory(traj.GetEnv(),'')
            self.traj.Init(traj.GetConfigurationSpecification())
            self.traj.Clone(traj,0)

        if waypoint_contact_manips is not None:
            self.waypoint_contact_manips = copy.deepcopy(waypoint_contact_manips)

        if path_generation_method is not None:
            self.path_generation_method = path_generation_method


def get_segment_explore_sequence(path_segments):

    segment_explore_sequence = []
    planning_segment_cache = []
    prev_path_generation_method = 'retrieval'

    for segment_index,path_segment in enumerate(path_segments):
        path_generation_method = path_segment.path_generation_method

        if prev_path_generation_method == 'retrieval' and path_generation_method == 'planning':
            planning_segment_cache.append(segment_index)
        elif prev_path_generation_method == 'retrieval' and path_generation_method == 'retrieval':
            segment_explore_sequence.append(segment_index)
        elif prev_path_generation_method == 'planning' and path_generation_method == 'planning':
            planning_segment_cache.append(segment_index)
        elif prev_path_generation_method == 'planning' and path_generation_method == 'retrieval':
            segment_explore_sequence.append(segment_index)
            segment_explore_sequence.extend(planning_segment_cache)
            planning_segment_cache = []

        prev_path_generation_method = path_generation_method

    segment_explore_sequence.extend(planning_segment_cache)

    return segment_explore_sequence


def main(contact_sequence_generation_method='all_planning',
         path_segmentation_type = 'motion_mode_and_traversability_segmentation',
         traversability_threshold_type='mean',
         traversability_threshold = 0.3,
         environment_path='environment_two_corridor',
         surface_source='two_corridor_environment',
         log_file_name='exp_result.txt',
         start_env_id=0,
         end_env_id=9999,
         recording_data=False,
         load_planning_result=False):

    rave.raveLogInfo('Using %s method...'%(contact_sequence_generation_method))

    if taking_planning_result_log:
        rave.raveLogInfo('Taking log. Store in %s.'%(log_file_name))


    # load the environment handler
    rave.raveLogInfo('Load the Environment Handler.')
    env_handler = environment_handler()
    env = env_handler.env
    structures = []

    ########################################################################
    # load and initialize the robot
    rave.raveLogInfo('Load and Initialize the Robot.')

    or_robot = load_escher.escher(env)

    ########################################################################
    # Load transition model

    # Load the hand transition model
    hand_transition_model = []
    hand_pitch = [-100.0,-90.0,-80.0,-70.0,-60.0,-50.0,-40.0,-30.0,-20.0,-10.0,0.0,10.0,20.0,30.0,40.0,50.0,60.0,70.0,80.0,90.0,100.0]
    hand_yaw = [-20.0,0.0,20.0]
    for pitch in hand_pitch:
        for yaw in hand_yaw:
            hand_transition_model.append((pitch,yaw))
    hand_transition_model.append((-99.0,-99.0))

    # Load the foot transition model
    try:
        print('Load step_transition_model...', end='')
        f = open(planning_data_path + 'step_transition_model_wide_range.txt','r')
        line = ' '
        step_transition_model = []

        while True:
            line = f.readline()
            if line == '':
                break

            step_transition_model.append((float(line[0:5]),float(line[6:11]),float(line[12:17])))

        f.close()
        print('Done.')
    except Exception:
        raw_input('Fail.')

    ########################################################################

    global contact_draw_handles

    DOFNameActiveIndexDict = or_robot.DOFNameActiveIndexDict
    IKInitDOFValues = or_robot.IKInitDOFValues
    StandingPostureDOFValues = or_robot.StandingPostureDOFValues

    # initialize the IK solver interface
    general_ik_interface = CBiRRT(env, or_robot.robot.GetName())

    ########################################################################
    rave.raveLogInfo('Load offline computed data.')
    # load other necessary offline computed data
    # load the footstep windows
    # Generated using transition_footstep_window_generator.py
    try:
        print('Load footstep_window...', end='')
        out_file = open(planning_data_path + 'footstep_window','r')
        footstep_windows = pickle.load(out_file)
        out_file.close()
        print('Done.')
    except Exception:
        raw_input('Fail.')
        return

    try:
        print('Load footstep_window_legs_only...', end='')
        out_file = open(planning_data_path + 'footstep_window_legs_only','r')
        footstep_windows_legs_only = pickle.load(out_file)
        out_file.close()
        print('Done.')
    except Exception:
        raw_input('Fail.')
        return

    # load motion plan library
    try:
        print('Load motion_plan_library...', end='')
        q_out_file = open(planning_data_path + 'motion_plan_library','r')
        motion_plan_library = pickle.load(q_out_file)
        q_out_file.close()
        print('Done.')
    except Exception:
        print('Not Found, use empty library.')
        motion_plan_library = []

    # load cluster file. (*)
    try:
        print('Load motion_plan_clusters...', end='')
        q_out_file = open(planning_data_path + 'motion_plan_clusters','r')
        motion_plan_clusters = pickle.load(q_out_file)
        q_out_file.close()
        print('Done.')
    except Exception:
        print('Not Found, treat the library as N cluster with each cluster contain 1 motion plan.')
        motion_plan_clusters = []
        if motion_plan_library:
            for i in range(len(motion_plan_library)):
                motion_plan_clusters.append((i,[i]))

    # load motion plan distance
    try:
        print('Load motion_plan_dist...', end='')
        out_file = open(planning_data_path + 'motion_plan_dist','r')
        motion_plan_dist = pickle.load(out_file)
        out_file.close()
        print('Done.')
    except Exception:
        print('Not Found, use empty plan_dist.')
        motion_plan_dist = {}

    # Load motion plan library/clusters/dist for each motion modes
    motion_plan_library_list = [[] for i in range(len(motion_mode_list))]
    motion_plan_clusters_list = [[] for i in range(len(motion_mode_list))]
    motion_plan_explore_order_list = [[] for i in range(len(motion_mode_list))]
    motion_plan_dist_list = [{} for i in range(len(motion_mode_list))]

    for i in range(len(motion_mode_list)):
        try:
            print('Load motion_plan_library for motion mode: %s...'%(motion_mode_list[i]), end='')
            q_out_file = open(planning_data_path + 'motion_plan_library_' + motion_mode_list[i],'r')
            motion_plan_library_list[i] = pickle.load(q_out_file)
            q_out_file.close()
            print('Done.')
        except Exception:
            print('Not Found, use empty library.')
            motion_plan_library_list[i] = []

        try:
            print('Load motion_plan_clusters for motion mode: %s...'%(motion_mode_list[i]), end='')
            q_out_file = open(planning_data_path + 'motion_plan_clusters_' + motion_mode_list[i],'r')
            motion_plan_clusters_list[i] = pickle.load(q_out_file)
            q_out_file.close()
            print('Done.')
        except Exception:
            print('Not Found, treat the library as N cluster with each cluster contain 1 motion plan.')
            motion_plan_clusters_list[i] = []
            if motion_plan_library_list[i]:
                for j in range(len(motion_plan_library_list[i])):
                    motion_plan_clusters_list[i].append((j,[j]))

        try:
            print('Load motion_plan_dist for motion mode: %s...'%(motion_mode_list[i]), end='')
            out_file = open(planning_data_path + 'motion_plan_dist_' + motion_mode_list[i],'r')
            motion_plan_dist_list[i] = pickle.load(out_file)
            out_file.close()
            print('Done.')
        except Exception:
            print('Not Found, use empty plan_dist.')
            motion_plan_dist_list[i] = {}

        try:
            print('Load motion_plan_explore_order for motion mode: %s...'%(motion_mode_list[i]), end='')
            out_file = open(planning_data_path + 'motion_plan_explore_order_' + motion_mode_list[i],'r')
            motion_plan_explore_order_list[i] = pickle.load(out_file)
            out_file.close()
            print('Done.')
        except Exception:
            if motion_plan_library_list[i]:
                print('Not Found, create one now.')
                update_motion_plan_explore_order(motion_plan_library_list[i],motion_plan_explore_order_list[i],i)
            else:
                print('Not Found, but motion plan library is also empty. Use empty explore sequence.')
                motion_plan_explore_order_list[i] = []


        if not recording_data:
            library_size = 50
            if len(motion_plan_library_list[i]) < library_size:
                rave.raveLogError('Motion plan library for %s motion mode is less than the specified library size %d.'%(motion_mode_list[i],library_size))
                return
            elif len(motion_plan_library_list[i]) > library_size:
                (motion_plan_library_list[i], motion_plan_dist_list[i], motion_plan_explore_order_list[i]) = trim_down_motion_plan_library(motion_plan_library_list[i],library_size,i)

    env_id = start_env_id

    ########################################################################
    rave.raveLogInfo('Start while loop to plan and collect plans.')
    # Start the while loop for planning
    while env_id <= end_env_id:

        RefreshHandler()

        # Remove the obstacles to avoid duplicated adding obstacles in next run
        for struct in structures:
            if struct.kinbody is not None:
                or_robot.env.Remove(struct.kinbody)
                struct.kinbody = None

        planning_healthy = True
        traj_feasible = False

        death_cause = '## '

        ########################################################################
        rave.raveLogInfo('Initialize the Environment.')

        # initialize the environment
        env_handler.update_environment(or_robot, planning_data_path + environment_path + '/env_' + str(env_id), surface_source)
        rave.raveLogInfo('Load environment: %s'%(planning_data_path + environment_path + '/env_' + str(env_id)))

        if load_planning_result:
            try:
                rave.raveLogInfo('Load planning result...', end='')
                out_file = open(planning_data_path + 'planning_result/planning_result_' + str(env_id),'r')
                planning_result = pickle.load(out_file)
                out_file.close()
                rave.raveLogInfo('Done.')
            except Exception:
                rave.raveLogInfo('Not Found, make it as None.')
                planning_result = None

        env_id += 1

        goal_x = env_handler.goal_x
        goal_y = env_handler.goal_y
        goal_theta = 0

        goal = [goal_x,goal_y,goal_theta]

        # set up the structures
        structures = env_handler.structures

        structures_dict = {}
        for struct in structures:
            structures_dict[struct.id] = struct

        #######################################################################
        # # collect test environment
        # rave.raveLogInfo('Store the Environment.')
        # # Store the environment

        # for struct in structures:
        #     or_robot.env.Remove(struct.kinbody)
        #     struct.kinbody = None

        # env_index = 0
        # structures_file_name = planning_data_path + 'environment/env_' + str(env_index)

        # while os.path.exists(structures_file_name):
        #     env_index = env_index + 1
        #     structures_file_name = planning_data_path + 'environment/env_' + str(env_index)

        # out_file = open(structures_file_name,'w')
        # pickle.dump((structures,goal_x,goal_y,goal_theta),out_file)
        # out_file.close()
        # continue
        ######################################################e#################


        rave.raveLogInfo('Initialize the Robot and the C++ Interface.')
        # iniitialize the robot
        or_robot.robot.SetDOFValues(IKInitDOFValues)

        # Initialize Elastic Strips interface
        elasticstrips = ElasticStrips(or_robot.env, or_robot.robot.GetName())

        # Initialize Escher C++ interface
        traversability_cpp = traversability_cpp_utility_wrapper(env)

        ########################################################################

        if not load_planning_result or planning_result is None:

            rave.raveLogInfo('Initialize the Map Grid.')

            start_time = time.time() ##! the start of every process

            if recording_data:
                planning_time_limit = None
            else:
                planning_time_limit = 500.0

            # initialize the map grid
            torso_pose_grid = map_grid(map_grid_resolution, structures, or_robot)

            # draw the goal
            (goal_nx,goal_ny,goal_ntheta) = torso_pose_grid.position_to_grid((goal_x,goal_y,goal_theta))
            DrawRegion(or_robot.env, xyzrpy_to_SE3([goal_x,goal_y,torso_pose_grid.cell_2D_list[goal_nx][goal_ny].height+0.02,0,0,goal_theta]), goal_br)

            torso_pose_grid_construction_time = time.time() - start_time ##! time used to construct the torso_pose_grid
            last_time_stamp = time.time()


            start_cpp_feature_vector_calculation = time.time()

            all_torso_transition = torso_pose_grid.get_all_torso_transition()

            get_all_torso_transition = time.time()

            (footstep_transition_traversability_legs_only,footstep_transition_traversability,hand_transition_traversability) = \
            traversability_cpp.SendStartCalculatingTraversabilityCommand(structures,
                                                                footstep_windows_legs_only,
                                                                footstep_windows,
                                                                all_torso_transition,# env_transition_checking_cells_cpp,
                                                                foot_contact_point_resolution,
                                                                torso_pose_grid,
                                                                hand_transition_model)

            end_cpp_feature_vector_calculation = time.time()

            torso_pose_grid.update_traversability_features(footstep_transition_traversability_legs_only,footstep_transition_traversability,hand_transition_traversability)

            end_copying_result = time.time()

            cpp_feature_vector_calculation_time = time.time() - last_time_stamp ##! time used to calculate traversability feature vectors

            # rave.raveLogInfo('Environment transition feature calculation time in cpp: %5.4f.'%(end_cpp_feature_vector_calculation-start_cpp_feature_vector_calculation))
            rave.raveLogInfo('Get All Torso Transition: %5.4f.'%(get_all_torso_transition-start_cpp_feature_vector_calculation))
            rave.raveLogInfo('CPP Traversability Calculation: %5.4f.'%(end_cpp_feature_vector_calculation-get_all_torso_transition))
            rave.raveLogInfo('Copying data to traversability feature vectors: %5.4f.'%(end_copying_result-end_cpp_feature_vector_calculation))
            rave.raveLogInfo('Torso Translation Num: %d.'%(len(all_torso_transition)))

            start_copy_time = time.time()

            torso_pose_grid_original = copy.deepcopy(torso_pose_grid)

            torso_pose_grid_copy_time = time.time() - start_copy_time
            last_time_stamp = time.time()

            #######################################################################
            rave.raveLogInfo('Run Dijkstra to get the Torso Policy and motion modes.')
            # Dijkstra path planning to get the torso policy

            start_dh_planning = time.time()
            torso_path = torso_pose_grid.path_planning(goal,start=(0,0,0),method='A_Star',direction='backward',exhaust_the_map=False,env_transition_bias=True,consider_motion_mode=True)
            dijkstra_map_constructed = torso_path
            end_dh_planning = time.time()

            torso_path_planning_time = time.time() - last_time_stamp ##! time used to calculate the torso path
            last_time_stamp = time.time()

            rave.raveLogInfo('Grid A_Star Planning Time: %5.4f.'%(end_dh_planning-start_dh_planning))

            if not dijkstra_map_constructed:
                rave.raveLogError('Dijkstra map fails.')
                continue

            #######################################################################
            # Find the optimal segmentation of the torso path
            rave.raveLogInfo('Find the torso path segmentation.')

            ##### Work on get_torso_path_segment if we want to further decompose this path to smaller segments #####
            torso_path_segmentation = get_torso_path_segmentation(torso_pose_grid,
                                                                  torso_path,
                                                                  path_segmentation_type=path_segmentation_type,
                                                                  traversability_threshold=traversability_threshold)
            num_segments = len(torso_path_segmentation[0])

            #######################################################################
            # Construct the initial node
            total_init_left_leg = [0.025,0.1,0,0,0,0]
            total_init_right_leg = [0.025,-0.1,0,0,0,0]
            total_init_left_arm = free_floating_pose
            total_init_right_arm = free_floating_pose
            total_init_manipulation_objects_configuration = None
            total_init_manipulating_objects = None
            total_initial_node = node(total_init_left_leg,total_init_right_leg,total_init_left_arm,total_init_right_arm,0,0,None,None)

            #######################################################################
            rave.raveLogInfo('Start to Generate the Path.')
            # Generating Path (Planning or query the library)
            path = [total_initial_node]

            rave_traj = rave.RaveCreateTrajectory(or_robot.env, '')
            rave_traj.Init(or_robot.robot.GetActiveConfigurationSpecification())

            waypoint_contact_manips = []

            # Decide the path generation method for each segment.
            path_segments = [None] * num_segments
            connected_path_segments = None

            for segment_index,torso_path_segment_indices_and_motion_mode in enumerate(torso_path_segmentation[0]):
                torso_path_segment_indices = torso_path_segment_indices_and_motion_mode[0:2]
                torso_path_segment_motion_mode = torso_path_segment_indices_and_motion_mode[2]
                new_path_segment = path_segment(torso_path_segment_indices,torso_path_segment_motion_mode)

                if contact_sequence_generation_method == 'all_planning':
                    new_path_segment.path_generation_method  = 'planning'

                elif contact_sequence_generation_method == 'all_retrieval':
                    new_path_segment.path_generation_method  = 'retrieval'

                elif contact_sequence_generation_method == 'hybrid':
                    new_torso_segment = torso_path[torso_path_segment_indices[0]:torso_path_segment_indices[1]+1]
                    segment_traversability_score_mean = get_torso_path_segment_traversability_score_mean(torso_pose_grid,
                                                                                                        new_torso_segment,
                                                                                                        torso_path_segment_motion_mode)

                    segment_traversability_score_max = get_torso_path_segment_traversability_score_max(torso_pose_grid,
                                                                                                    new_torso_segment,
                                                                                                    torso_path_segment_motion_mode)

                    if traversability_threshold_type == 'mean':
                        if segment_traversability_score_mean >= traversability_threshold:
                            new_path_segment.path_generation_method  = 'retrieval'
                        else:
                            new_path_segment.path_generation_method  = 'planning'

                    elif traversability_threshold_type == 'max':
                        if segment_traversability_score_max >= traversability_threshold:
                            new_path_segment.path_generation_method = 'retrieval'
                        else:
                            new_path_segment.path_generation_method = 'planning'

                    rave.raveLogInfo('Segment %d: score_mean: %5.3f, max_score: %5.3f. Path Generation Method: %s.'%(segment_index,
                                                                                                    segment_traversability_score_mean,
                                                                                                    segment_traversability_score_max,
                                                                                                    new_path_segment.path_generation_method))

                for i in range(torso_path_segment_indices[0],torso_path_segment_indices[1]+1):
                    if i != torso_path_segment_indices[0]:
                        if path_segmentation_type == 'no_segmentation':
                            DrawGridPath([torso_path[i].get_position(),torso_path[i-1].get_position()], env, torso_pose_grid,
                                          [0,0,0],z=1.4,style='single')
                        else:
                            if new_path_segment.path_generation_method  == 'planning':
                                DrawGridPath([torso_path[i].get_position(),torso_path[i-1].get_position()], env, torso_pose_grid,
                                            motion_mode_color_dict[torso_path_segment_motion_mode],z=1.4,style='single')
                            elif new_path_segment.path_generation_method  == 'retrieval':
                                DrawGridPath([torso_path[i].get_position(),torso_path[i-1].get_position()], env, torso_pose_grid,
                                            motion_mode_color_dict[torso_path_segment_motion_mode],z=1.4,style='single')

                # DrawPoint(env,(torso_path[torso_path_segment_indices[1]].get_position()[0],torso_path[torso_path_segment_indices[1]].get_position()[1],0.2),rgb=[193.0/255.0,50.0/255.0,142.0/255.0,0.8],size=0.2)
                DrawPoint(env,(torso_path[torso_path_segment_indices[1]].get_position()[0],torso_path[torso_path_segment_indices[1]].get_position()[1],1.4),rgb=[193.0/255.0,50.0/255.0,142.0/255.0,0.8],size=0.2)

                path_segments[segment_index] = new_path_segment


            torso_path_segmentation_time = time.time() - last_time_stamp ##! time used to segment the torso path
            last_time_stamp = time.time()

            #######################################################################
            # Find the contact regions

            rave.raveLogInfo('Find the contact regions.')

            # (contact_points_values, contact_regions_values) = traversability_cpp.SendStartConstructingContactRegions(printing=False, structures_id=list(neighbor_structure_ids))
            (contact_points_values, contact_regions_values) = traversability_cpp.SendStartConstructingContactRegions(printing=False)

            contact_regions = []
            f_contact_region = open('contact_regions.txt','w')

            for i in range(0,len(contact_regions_values)):
                cr_position = np.array([[contact_regions_values[i][0]],[contact_regions_values[i][1]],[contact_regions_values[i][2]]])
                cr_normal = np.array([[contact_regions_values[i][3]],[contact_regions_values[i][4]],[contact_regions_values[i][5]]])
                cr_radius = contact_regions_values[i][6]
                contact_regions.append(contact_region(cr_position, cr_normal, cr_radius))
                # DrawContactRegion(env, cr_position, cr_normal, cr_radius)
                f_contact_region.write('%5.5f %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f\n'%(cr_position[0,0],cr_position[1,0],cr_position[2,0],cr_normal[0,0],cr_normal[1,0],cr_normal[2,0],cr_radius))

            f_contact_region.close()

            # making the contact points list
            contact_points = []
            for i in range(0,len(contact_points_values)):
                cp_position = np.array(contact_points_values[i][0:3])
                cp_normal = np.array(contact_points_values[i][3:6])

                if cp_position[2] < 0.4:
                    contact_points.append(contact_point(cp_position,cp_normal,0))
                else:
                    contact_points.append(contact_point(cp_position,cp_normal,1))

            contact_region_generation_time = time.time() - last_time_stamp ##! time used to generate contact regions and contact points
            last_time_stamp = time.time()

            #######################################################################
            # Generate path for each segment

            rave.raveLogInfo('Generating Path for each Segment.')
            add_path_subsegments = False
            path_subsegment_list = []
            segment_break_index = 0

            while any([segment.contact_state_path is None for segment in path_segments]):

                # add the subsegments into the path segments
                if add_path_subsegments:
                    rave.raveLogInfo('add path subsegment!!')
                    path_segments = path_segments[0:segment_break_index] + path_subsegment_list + path_segments[segment_break_index+1:]
                    add_path_subsegments = False

                num_segments = len(path_segments)

                if planning_time_limit is not None and start_time+planning_time_limit-torso_pose_grid_copy_time-time.time() < 0:
                    rave.raveLogError('Time Out!')
                    planning_healthy = False
                    break

                # Decide the explore sequence for the path segments
                segment_explore_sequence = get_segment_explore_sequence(path_segments)

                for segment_index in segment_explore_sequence:

                    if planning_time_limit is not None and start_time+planning_time_limit-torso_pose_grid_copy_time-time.time() < 0:
                        planning_healthy = False
                        break

                    segment = path_segments[segment_index]

                    if segment_index > 0:
                        prev_segment = path_segments[segment_index-1]

                    if segment_index < num_segments-1:
                        next_segment = path_segments[segment_index+1]

                    if segment.contact_state_path is not None:
                        continue

                    torso_path_segment_indices = segment.torso_path_boundary_indices

                    start_torso_state = torso_path[torso_path_segment_indices[0]]
                    goal_torso_state = torso_path[torso_path_segment_indices[1]]
                    torso_path_segment = torso_path[torso_path_segment_indices[0]:torso_path_segment_indices[1]+1]

                    segment_goal = (goal_torso_state.x,goal_torso_state.y,goal_torso_state.theta)
                    segment_start = (start_torso_state.x,start_torso_state.y,start_torso_state.theta)

                    or_robot.robot.SetDOFValues(StandingPostureDOFValues)

                    path_generation_method = segment.path_generation_method
                    motion_mode = segment.motion_mode

                    if path_generation_method == 'retrieval': # Use RA

                        rave.raveLogInfo('Now Start Matching Motion Plan to Segment %d.'%(segment_index))

                        torso_pose_grid_segment_goal = torso_pose_grid_original
                        torso_path_segment_neighbor_indices = torso_pose_grid_segment_goal.get_neighbor_cell_indices(torso_path_segment)

                        local_torso_path_segment = torso_pose_grid_segment_goal.path_planning(segment_goal,start=segment_start,method='A_Star',direction='backward',exhaust_the_map=False,search_space=torso_path_segment_neighbor_indices)

                        motion_plan_library_mm = motion_plan_library_list[motion_mode]
                        motion_plan_clusters_mm = motion_plan_clusters_list[motion_mode]
                        motion_plan_explore_order_mm = motion_plan_explore_order_list[motion_mode]

                        matching_result = plan_env_matching(or_robot,
                                                            contact_points,
                                                            contact_regions,
                                                            torso_pose_grid,
                                                            segment_start,
                                                            segment_goal,
                                                            local_torso_path_segment, #torso_path_segment
                                                            structures,
                                                            motion_mode,
                                                            motion_plan_library=motion_plan_library_mm,
                                                            motion_plan_clusters=motion_plan_clusters_mm,
                                                            motion_plan_explore_order=motion_plan_explore_order_mm,
                                                            planning_time_limit=start_time+planning_time_limit-torso_pose_grid_copy_time-time.time(),
                                                            elasticstrips=elasticstrips,
                                                            general_ik_interface=general_ik_interface)

                        torso_pose_grid_segment_goal.reset_cells_cost()

                        if matching_result is None:
                            rave.raveLogWarn('No Matching Motion Plan, Switch to Planning and Reorder the Segments Exploration Sequence.')

                            death_cause += ('library fail in segment %d;'%(segment_index))

                            segment.path_generation_method = 'planning'
                            # segment_explore_sequence = get_segment_explore_sequence(path_segments)
                            break
                        else:
                            if len(matching_result) == 1: # when there is only 1 subsegment
                                segment_path = matching_result[0][0]
                                segment_rave_traj = matching_result[0][1]
                                segment_waypoint_contact_manips = matching_result[0][2]

                                # segment.update_path_segment(segment_path, segment_rave_traj, segment_waypoint_contact_manips)
                                segment.update_path_segment(contact_state_path=segment_path)

                                DrawStances(segment_path[-1],or_robot,env,contact_draw_handles,marker='cavalry')

                                rave.raveLogInfo('Matched Motion Plan Found: %d/%d of Segments are Generated.'
                                    %(sum([segment.contact_state_path is not None for segment in path_segments]),num_segments))
                            else:
                                segment_break_index = segment_index
                                path_subsegment_list = []

                                for subsegment_result in matching_result:
                                    subsegment_path = subsegment_result[0]
                                    subsegment_rave_traj = subsegment_result[1]
                                    subsegment_waypoint_contact_manips = subsegment_result[2]
                                    subsegment_torso_path_indices = (subsegment_result[3][0]+torso_path_segment_indices[0],subsegment_result[3][1]+torso_path_segment_indices[0])

                                    if subsegment_path is not None:

                                        DrawStances(subsegment_path[-1],or_robot,env,contact_draw_handles,marker='cavalry')

                                        new_path_subsegment = path_segment(subsegment_torso_path_indices,motion_mode)

                                        new_path_subsegment.update_path_segment(contact_state_path=subsegment_path, path_generation_method='retrieval')

                                    else:
                                        new_path_subsegment = path_segment(subsegment_torso_path_indices,motion_mode)
                                        new_path_subsegment.update_path_segment(contact_state_path=None, path_generation_method='planning')


                                    path_subsegment_list.append(new_path_subsegment)

                                add_path_subsegments = True
                                rave.raveLogInfo('%d subsegments is added in replace of segment %d.'%(len(path_subsegment_list),segment_index))

                                break

                    elif path_generation_method == 'planning': # Use PFS

                        # Planning
                        rave.raveLogInfo('Constructing segment goal Dijkstra map...')
                        torso_pose_grid_segment_goal = torso_pose_grid_original
                        torso_path_segment_neighbor_indices = torso_pose_grid_segment_goal.get_neighbor_cell_indices(torso_path_segment)
                        # DrawCells(env, torso_path_segment_neighbor_indices, torso_pose_grid_segment_goal)

                        rave.raveLogInfo('Now Start Planning...')
                        planning_starting_time = time.time()

                        rave.raveLogInfo('Segment Goal: (%5.3f,%5.3f,%5.3f), Motion Mode: %s.'%(segment_goal[0],segment_goal[1],segment_goal[2],motion_mode_list[motion_mode]))

                        # record the final node
                        if segment_index == 0:
                            last_motion_mode = None
                            initial_node = copy.deepcopy(total_initial_node)
                        else:
                            if prev_segment.contact_state_path is not None:
                                last_motion_mode = prev_segment.motion_mode
                            else:
                                last_motion_mode = None
                            initial_node = copy.deepcopy(prev_segment.contact_state_path[-1])

                        initial_node.explore_state = 'OPEN'
                        initial_node.parent = None
                        initial_node.g = 0
                        initial_node.left_arm_moved = False
                        initial_node.right_arm_moved = False

                        # final segment or the next segment is also using planning
                        if segment_index == (num_segments-1) or next_segment.contact_state_path is None:
                            dijkstra_map_constructed = torso_pose_grid_segment_goal.path_planning(segment_goal,start=None,method='Dijkstra',direction='backward',exhaust_the_map=True,search_space=torso_path_segment_neighbor_indices)
                            # DrawRegion(or_robot.env,xyzrpy_to_SE3([segment_goal[0],segment_goal[1],0.1,0,0,segment_goal[2]]),goal_br)

                            initial_node.h = h_estimation(initial_node.left_leg,
                                                          initial_node.right_leg,
                                                          initial_node.left_arm,
                                                          initial_node.right_arm,
                                                          or_robot,
                                                          torso_pose_grid_segment_goal,
                                                          goal=segment_goal,
                                                          motion_mode=motion_mode,
                                                          heuristics='dijkstra')

                            if recording_data:
                                segment_path = ANA_Star(general_ik_interface, or_robot, structures, torso_pose_grid_segment_goal,
                                                        goal=segment_goal, initial_node=initial_node,
                                                        step_transition_model=step_transition_model,
                                                        hand_transition_model=hand_transition_model,
                                                        motion_mode=motion_mode,
                                                        other_motion_modes=[last_motion_mode],
                                                        heuristics='dijkstra',
                                                        goal_radius=goal_br,
                                                        planning_time_limit=ANA_time_limit,
                                                        preferred_planning_time=min(ANA_time_limit-1.0,len(torso_path_segment)*5.0))
                            else:
                                if path_segmentation_type == 'no_segmentation':
                                    segment_path = ANA_Star(general_ik_interface, or_robot, structures, torso_pose_grid_segment_goal,
                                                            goal=segment_goal, initial_node=initial_node,
                                                            step_transition_model=step_transition_model,
                                                            hand_transition_model=hand_transition_model,
                                                            motion_mode=3,
                                                            other_motion_modes=[3,0,1,2],
                                                            heuristics='dijkstra',
                                                            goal_radius=goal_br,
                                                            planning_time_limit=start_time+planning_time_limit-torso_pose_grid_copy_time-time.time(),
                                                            multiple_motion_modes=True)
                                else:

                                    other_motion_modes = [last_motion_mode]

                                    segment_path = ANA_Star(general_ik_interface, or_robot, structures, torso_pose_grid_segment_goal,
                                                            goal=segment_goal, initial_node=initial_node,
                                                            step_transition_model=step_transition_model,
                                                            hand_transition_model=hand_transition_model,
                                                            motion_mode=motion_mode,
                                                            other_motion_modes=other_motion_modes,
                                                            heuristics='dijkstra',
                                                            goal_radius=goal_br,
                                                            planning_time_limit=start_time+planning_time_limit-torso_pose_grid_copy_time-time.time())

                                    if not segment_path:
                                        death_cause += ('normal planning time out in segment %d;'%(segment_index))

                        # the next segment is using motion plan library query
                        else:
                            segment_goal_node = next_segment.contact_state_path[0]
                            goal_pose = [segment_goal_node.left_leg, segment_goal_node.right_leg,
                                         segment_goal_node.left_arm, segment_goal_node.right_arm]
                            segment_goal_pose = segment_goal_node.get_mean_feet_xyzrpy()
                            tentative_segment_goal = [segment_goal_pose[0],segment_goal_pose[1],segment_goal_pose[5]] # try to set the goal to where the contact pose is at

                            dijkstra_map_constructed = torso_pose_grid_segment_goal.path_planning(tentative_segment_goal,start=None,method='Dijkstra',direction='backward',exhaust_the_map=True,search_space=torso_path_segment_neighbor_indices)

                            segment_goal = tentative_segment_goal

                            if not dijkstra_map_constructed:
                                rave.raveLogInfo('Use Euclidean heuristics')
                                heuristics = 'euclidean'
                            else:
                                rave.raveLogInfo('Use Dijkstra heuristics')
                                heuristics = 'dijkstra'

                            initial_node.h = h_estimation(initial_node.left_leg,
                                                          initial_node.right_leg,
                                                          initial_node.left_arm,
                                                          initial_node.right_arm,
                                                          or_robot,
                                                          torso_pose_grid_segment_goal,
                                                          goal=segment_goal,
                                                          motion_mode=motion_mode,
                                                          heuristics=heuristics,
                                                          exact_goal_pose=goal_pose)

                            other_motion_modes = [last_motion_mode]

                            if recording_data:
                                segment_path = ANA_Star(general_ik_interface, or_robot, structures, torso_pose_grid_segment_goal,
                                                        goal=segment_goal, initial_node=initial_node,
                                                        step_transition_model=step_transition_model,
                                                        hand_transition_model=hand_transition_model,
                                                        motion_mode=motion_mode,
                                                        other_motion_modes=other_motion_modes,
                                                        heuristics=heuristics,
                                                        exact_goal_pose=goal_pose,
                                                        planning_time_limit=ANA_time_limit,#planning_time_limit=start_time+planning_time_limit-torso_pose_grid_copy_time-time.time())#,
                                                        preferred_planning_time=min(ANA_time_limit-1.0,len(torso_path_segment)*5.0))
                            else:
                                segment_path = ANA_Star(general_ik_interface, or_robot, structures, torso_pose_grid_segment_goal,
                                                        goal=segment_goal, initial_node=initial_node,
                                                        step_transition_model=step_transition_model,
                                                        hand_transition_model=hand_transition_model,
                                                        motion_mode=motion_mode,
                                                        other_motion_modes=other_motion_modes,
                                                        heuristics=heuristics,
                                                        exact_goal_pose=goal_pose,
                                                        planning_time_limit=start_time+planning_time_limit-torso_pose_grid_copy_time-time.time())

                                if not segment_path:
                                    death_cause += ('exact_goal_pose planning time out in segment %d;'%(segment_index))

                        planning_time = time.time() - planning_starting_time

                        torso_pose_grid_segment_goal.reset_cells_cost()

                        if len(segment_path) != 0:
                            rave.raveLogInfo('Planning Done. ANA* Planning Time: %5.5f'%(planning_time))
                        else:
                            rave.raveLogError('Planning Fail. ANA* Planning Time: %5.5f'%(planning_time))
                            planning_healthy = False
                            rave.raveLogError('Cannot find a solution trajectory for this environment.')
                            break

                        # update the segment path
                        segment.update_path_segment(contact_state_path=segment_path)

                        rave.raveLogInfo('Motion Plan Generated: %d/%d of Segments are Generated.'
                            %(sum([s.contact_state_path is not None for s in path_segments]),num_segments))


                if planning_healthy == False:
                    if planning_time_limit is not None and start_time+planning_time_limit-torso_pose_grid_copy_time-time.time() < 0:
                        rave.raveLogError('Time Out!')
                    break

            segment_contact_planning_time = time.time() - last_time_stamp ##! planning time for all segments
            last_time_stamp = time.time()

            # reconnect the connection part with small planning
            if planning_healthy:
                connected_path_segments = []
                for segment_index in range(num_segments):

                    segment = path_segments[segment_index]

                    if segment_index > 0:
                        prev_segment = path_segments[segment_index-1]

                    if segment_index < num_segments-1:
                        next_segment = path_segments[segment_index+1]

                    if segment.path_generation_method == 'retrieval' and (segment_index == 0 or prev_segment.path_generation_method == 'retrieval'):

                        # start planning
                        if segment_index == 0:
                            rave.raveLogInfo('Now Connecting Initial Stance and Segment 0...')
                        else:
                            rave.raveLogInfo('Now Connecting Segment %d and %d...'%(segment_index-1,segment_index))

                        # set the initial node of the planning as the final node of the last segment
                        if segment_index == 0:
                            last_motion_mode = None
                            initial_node = copy.deepcopy(total_initial_node)
                        else:
                            last_motion_mode = prev_segment.motion_mode
                            initial_node = copy.deepcopy(prev_segment.contact_state_path[-1])

                        # set the goal node of the planning as the initial node of the current segment
                        segment_init_node = segment.contact_state_path[0]
                        connection_goal_node = segment_init_node

                        connection_goal_pose = [connection_goal_node.left_leg, connection_goal_node.right_leg,
                                                connection_goal_node.left_arm, connection_goal_node.right_arm]

                        connection_goal = ((connection_goal_node.left_leg[0]+connection_goal_node.right_leg[0])/2.0,
                                        (connection_goal_node.left_leg[1]+connection_goal_node.right_leg[1])/2.0,
                                        connection_goal_node.get_virtual_body_yaw())

                        initial_node.explore_state = 'OPEN'
                        initial_node.parent = None
                        initial_node.g = 0
                        initial_node.h = h_estimation(initial_node.left_leg,
                                                      initial_node.right_leg,
                                                      initial_node.left_arm,
                                                      initial_node.right_arm,
                                                      or_robot,
                                                      torso_pose_grid_original,
                                                      goal=connection_goal,
                                                      motion_mode=segment.motion_mode,
                                                      heuristics='euclidean',
                                                      exact_goal_pose=connection_goal_pose)

                        segment_path = ANA_Star(general_ik_interface, or_robot, structures, torso_pose_grid_original,
                                                goal=connection_goal, initial_node=initial_node,
                                                step_transition_model=step_transition_model,
                                                hand_transition_model=hand_transition_model,
                                                motion_mode=segment.motion_mode,
                                                other_motion_modes=[last_motion_mode],
                                                heuristics='euclidean',
                                                exact_goal_pose=connection_goal_pose,
                                                planning_time_limit=start_time+planning_time_limit-torso_pose_grid_copy_time-time.time(),
                                                skip_state_feasibility_test=True,
                                                initial_G_h_ratio=2.0)

                        if not segment_path:
                            death_cause += ('connection planning time out between segment %d and %d;'%(segment_index-1,segment_index))

                        if len(segment_path) != 0:
                            rave.raveLogInfo('Connection Planning Done.')
                        else:
                            rave.raveLogError('Connection Planning Fail.')
                            planning_healthy = False
                            if planning_time_limit is not None and start_time+planning_time_limit-torso_pose_grid_copy_time-time.time() < 0:
                                rave.raveLogError('Time Out!')
                            rave.raveLogError('Cannot find a solution trajectory for this environment.')
                            break

                        new_path_segment = path_segment((segment.torso_path_boundary_indices[0],segment.torso_path_boundary_indices[0]),'all_manipulators')

                        new_path_segment.update_path_segment(contact_state_path=segment_path)

                        connected_path_segments.append(new_path_segment)

                        if segment_index == 0:
                            rave.raveLogInfo('Initial stance and %d are connected.'%(segment_index))
                        else:
                            rave.raveLogInfo('Segments %d and %d are connected.'%(segment_index-1,segment_index))

                    connected_path_segments.append(segment)

            connecting_contact_planning_time = time.time() - last_time_stamp ##! planning time for connecting segments

            if planning_healthy and planning_time_limit is not None and start_time+planning_time_limit-torso_pose_grid_copy_time-time.time() < 0:
                rave.raveLogError('Time Out!')
                planning_healthy = False

            ## log the timing result
            if taking_planning_result_log:
                if planning_healthy:
                    planning_success = 1
                else:
                    planning_success = 0

                num_segments_completed = sum([s.contact_state_path is not None for s in path_segments])
                num_segment_completed_using_planning = sum([(s.contact_state_path is not None and s.path_generation_method == 'planning') for s in path_segments])
                num_segment_completed_using_retrieval = sum([(s.contact_state_path is not None and s.path_generation_method == 'retrieval') for s in path_segments])


                result_outfile = open(log_file_name,'a')
                result_outfile.write('%d %d %d %d %d %d %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %s\n'%(
                                                                            env_id-1,
                                                                            planning_success,
                                                                            num_segments_completed,
                                                                            num_segments,
                                                                            num_segment_completed_using_planning,
                                                                            num_segment_completed_using_retrieval,
                                                                            torso_pose_grid_construction_time,
                                                                            cpp_feature_vector_calculation_time,
                                                                            torso_path_planning_time,
                                                                            torso_path_segmentation_time,
                                                                            contact_region_generation_time,
                                                                            segment_contact_planning_time,
                                                                            connecting_contact_planning_time,
                                                                            death_cause))
                result_outfile.close()

                planning_result_outfile = open(planning_data_path + 'planning_result/planning_result_' + str(env_id-1),'w')
                pickle.dump(connected_path_segments,planning_result_outfile)
                planning_result_outfile.close()

        else:
            connected_path_segments = planning_result
            path = [copy.deepcopy(connected_path_segments[0].contact_state_path[0])]

            rave_traj = rave.RaveCreateTrajectory(or_robot.env, '')
            rave_traj.Init(or_robot.robot.GetActiveConfigurationSpecification())

            waypoint_contact_manips = []

            for segment in connected_path_segments:

                if segment.path_generation_method == 'retrieval':
                    DrawStances(segment.contact_state_path[-1],or_robot,env,contact_draw_handles,marker='cavalry')
                else:
                    DrawStances(segment.contact_state_path[-1],or_robot,env,contact_draw_handles)


        ################################################################################
        # step interpolation

        if generate_trajectory:

            if planning_healthy or recording_data: # only interpolate steps when the planning is still healthy or we are taking trajectory data

                rave.raveLogInfo('Contact space planning is done. Press Enter to interpolate contacts with the quasi-static trajectory.')

                raw_input()

                rave.raveLogInfo('Step Interpolation to get the Trajectory.')
                # Interpolate the plan to a trajectory containing series of poses.

                if connected_path_segments is not None:
                    target_path_segments = connected_path_segments
                else:
                    target_path_segments = path_segments

                for segment_index, segment in enumerate(target_path_segments):

                    segment_path = segment.contact_state_path

                    if segment_path is not None:
                        segment_rave_traj = rave.RaveCreateTrajectory(or_robot.env, '')
                        segment_rave_traj.Init(or_robot.robot.GetActiveConfigurationSpecification())

                        trajectory_interpolation_start_time = time.time()

                        if segment_index == 0 or recording_data:
                            reset_robot_config = True
                        else:
                            reset_robot_config = False

                        # with env:
                        (traj_feasible,segment_waypoint_contact_manips) = step_interpolation(segment_rave_traj, segment_path, or_robot, reset_robot_config)

                        trajectory_interpolation_time = time.time() - trajectory_interpolation_start_time

                        segment.update_path_segment(traj=segment_rave_traj, waypoint_contact_manips=segment_waypoint_contact_manips)

                        rave.raveLogInfo('Segment %d, Contact State Path Length: %d, Trajectory Interpolation Time: %5.3f'%(segment_index,len(segment_path),trajectory_interpolation_time))
                    else:
                        rave.raveLogInfo('Segment %d does not have contact state path, continue.'%(segment_index))


            ################################################################################
            # Stitch motion plan segments

            if planning_healthy:
                for segment_index,segment in enumerate(connected_path_segments):
                    segment_contact_state_path = segment.contact_state_path
                    segment_rave_traj = segment.traj
                    segment_waypoint_contact_manips = copy.deepcopy(segment.waypoint_contact_manips)

                    # the only state in the segment is the same as the last state in the last segment
                    if len(segment_contact_state_path) < 2:
                        continue

                    # connect contact path
                    segment_contact_state_path[1].parent = path[-1]
                    path += segment_contact_state_path[1:]

                    # connect trajectory
                    num_segment_waypoints = segment_rave_traj.GetNumWaypoints()
                    for j in range(num_segment_waypoints):
                        rave_traj.Insert(rave_traj.GetNumWaypoints(),segment_rave_traj.GetWaypoint(j))

                    # connect waypoint_contact_manips
                    if segment_index != 0:
                        # modify the segment_waypoint_contact_manip number
                        last_waypoint_contact_manips = waypoint_contact_manips[-1]

                        last_waypoint_contact_manips_dict = {}
                        start_manip_group_index = sys.maxint

                        for contact_manip in last_waypoint_contact_manips:
                            last_waypoint_contact_manips_dict[contact_manip[0]] = contact_manip[1]

                            if contact_manip[1] < start_manip_group_index:
                                start_manip_group_index = contact_manip[1]

                        manip_group_index_mapping = {} # segment manip_group index : total_path manip_group index

                        for waypoint_index in range(len(segment_waypoint_contact_manips)):
                            contact_manips = segment_waypoint_contact_manips[waypoint_index]

                            if waypoint_index == 0: # first construct the mapping of index for the first waypoint
                                for manip_name_group_index_pair in contact_manips:
                                    manip_name = manip_name_group_index_pair[0]
                                    manip_group_index = manip_name_group_index_pair[1]
                                    manip_group_index_mapping[manip_group_index] = last_waypoint_contact_manips_dict[manip_name]

                            for manip_index in range(len(contact_manips)):
                                manip_name_group_index_pair = contact_manips[manip_index]

                                manip_name = manip_name_group_index_pair[0]
                                manip_group_index = manip_name_group_index_pair[1]

                                if manip_group_index in manip_group_index_mapping:
                                    manip_name_group_index_pair = (manip_name, manip_group_index_mapping[manip_group_index])
                                else:
                                    manip_name_group_index_pair = (manip_name, manip_group_index + start_manip_group_index)

                    waypoint_contact_manips.extend(segment_waypoint_contact_manips)

                num_draw_handles = len(contact_draw_handles)
                for k in range(num_draw_handles):
                    del contact_draw_handles[-1]

                #######################################################################
                # Execution

                if len(path) != 0:
                    # Retime Trajectory
                    rave.planningutils.RetimeActiveDOFTrajectory(rave_traj,or_robot.robot,hastimestamps=False,maxvelmult=0.6,maxaccelmult=0.12)

                    # Store Trajectory
                    # f_traj_out = open('trajectory.xml','w')
                    # obj = rave_traj.serialize(0)
                    # f_traj_out.write(obj)
                    # f_traj_out.close()

                    # Show Trajectory
                    or_robot.robot.GetController().SetPath(rave_traj)
                    waitrobot(or_robot.robot)
                    IPython.embed()
                else:
                    rave.raveLogInfo("Empty path, cannot execute.")


            rave.raveLogInfo('Collect the Motion Plan.')
            # Collect the resulting motion plan

            if recording_data:
                for segment in path_segments:

                    if segment.contact_state_path is None or len(segment.contact_state_path) < 2:
                        continue

                    segment_path = copy.deepcopy(segment.contact_state_path)
                    segment_rave_traj = segment.traj
                    segment_waypoint_contact_manips = copy.deepcopy(segment.waypoint_contact_manips)

                    # omit those connecting contacts that do not belong to the motion type
                    contact_state_bound_indices = [None,len(segment_path)-1]
                    for i,segment_node in enumerate(segment_path):
                        if segment.motion_mode == 0:
                            matching_motion_mode = (segment_node.left_arm[0] != -99.0 and segment_node.right_arm[0] != -99.0)
                        elif segment.motion_mode == 1:
                            matching_motion_mode = (segment_node.left_arm[0] != -99.0 and segment_node.right_arm[0] == -99.0)
                        elif segment.motion_mode == 2:
                            matching_motion_mode = (segment_node.left_arm[0] == -99.0 and segment_node.right_arm[0] != -99.0)
                        elif segment.motion_mode == 3:
                            matching_motion_mode = (segment_node.left_arm[0] == -99.0 and segment_node.right_arm[0] == -99.0)

                        if contact_state_bound_indices[0] is None:
                            if matching_motion_mode:
                                contact_state_bound_indices[0] = i
                        else:
                            if not matching_motion_mode:
                                contact_state_bound_indices[1] = i-1
                                break

                    if contact_state_bound_indices[0] is None:
                        rave.raveLogInfo('Motion plan does not have matching motion mode node, do not store.')
                        continue

                    if contact_state_bound_indices[0] != 0 or contact_state_bound_indices[1] != len(segment_path)-1:
                        (segment_path,segment_traj_list,segment_waypoint_contact_manips) = extract_partial_motion_plan(segment_path,segment_rave_traj,
                                                                                                                    segment_waypoint_contact_manips,
                                                                                                                    contact_state_bound_indices)

                        segment_path[0].parent = None
                        segment_path[0].prev_move_manip = None

                        segment_rave_traj = rave.RaveCreateTrajectory(or_robot.env,'')
                        segment_rave_traj.Init(or_robot.robot.GetActiveConfigurationSpecification())

                        for q in segment_traj_list:
                            segment_rave_traj.Insert(segment_rave_traj.GetNumWaypoints(),q)


                    aligned_rave_traj = rave.RaveCreateTrajectory(or_robot.env,'')
                    aligned_rave_traj.Init(or_robot.robot.GetActiveConfigurationSpecification())

                    aligned_path = []
                    contact_sequence = []

                    # data alignment, find dtheta
                    init_foot_mean_x = (segment_path[0].left_leg[0]+segment_path[0].right_leg[0])/2.0
                    init_foot_mean_y = (segment_path[0].left_leg[1]+segment_path[0].right_leg[1])/2.0
                    init_foot_mean_z = (segment_path[0].left_leg[2]+segment_path[0].right_leg[2])/2.0

                    goal_foot_mean_x = (segment_path[-1].left_leg[0]+segment_path[-1].right_leg[0])/2.0
                    goal_foot_mean_y = (segment_path[-1].left_leg[1]+segment_path[-1].right_leg[1])/2.0
                    dtheta = math.atan2(goal_foot_mean_y-init_foot_mean_y,goal_foot_mean_x-init_foot_mean_x)
                    plan_travel_dist = math.hypot(goal_foot_mean_x-init_foot_mean_x,goal_foot_mean_y-init_foot_mean_y)

                    inverse_aligned_coordinate_transform = xyzrpy_to_inverse_SE3([init_foot_mean_x,init_foot_mean_y,init_foot_mean_z,0.0,0.0,dtheta*rad_to_deg])

                    for i in range(segment_rave_traj.GetNumWaypoints()):
                        q = copy.deepcopy(segment_rave_traj.GetWaypoint(i)[0:or_robot.robot.GetActiveDOF()])

                        affine_x = q[DOFNameActiveIndexDict['x_prismatic_joint']]
                        affine_y = q[DOFNameActiveIndexDict['y_prismatic_joint']]
                        affine_z = q[DOFNameActiveIndexDict['z_prismatic_joint']]
                        affine_roll = q[DOFNameActiveIndexDict['roll_revolute_joint']] * rad_to_deg
                        affine_pitch = q[DOFNameActiveIndexDict['pitch_revolute_joint']] * rad_to_deg
                        affine_yaw = q[DOFNameActiveIndexDict['yaw_revolute_joint']] * rad_to_deg

                        affine_transform = xyzrpy_to_SE3([affine_x,affine_y,affine_z,affine_roll,affine_pitch,affine_yaw])

                        new_affine_transform = np.dot(inverse_aligned_coordinate_transform,affine_transform)

                        new_affine_xyzrpy = SE3_to_xyzrpy(new_affine_transform)

                        q[DOFNameActiveIndexDict['x_prismatic_joint']] = new_affine_xyzrpy[0]
                        q[DOFNameActiveIndexDict['y_prismatic_joint']] = new_affine_xyzrpy[1]
                        q[DOFNameActiveIndexDict['z_prismatic_joint']] = new_affine_xyzrpy[2]
                        q[DOFNameActiveIndexDict['roll_revolute_joint']] = new_affine_xyzrpy[3] * deg_to_rad
                        q[DOFNameActiveIndexDict['pitch_revolute_joint']] = new_affine_xyzrpy[4] * deg_to_rad
                        q[DOFNameActiveIndexDict['yaw_revolute_joint']] = new_affine_xyzrpy[5] * deg_to_rad

                        aligned_rave_traj.Insert(aligned_rave_traj.GetNumWaypoints(),q)

                    for i,stance in enumerate(segment_path):
                        if i == 0:
                            init_left_foot_contact = contact(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.left_leg)),'l_foot')
                            contact_sequence.append(init_left_foot_contact)

                            init_right_foot_contact = contact(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.right_leg)),'r_foot')
                            contact_sequence.append(init_right_foot_contact)

                            if stance.left_arm[0] != -99.0:
                                init_left_hand_contact = contact(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.left_arm)),'l_hand')
                                contact_sequence.append(init_left_hand_contact)

                            if stance.right_arm[0] != -99.0:
                                init_right_hand_contact = contact(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.right_arm)),'r_hand')
                                contact_sequence.append(init_right_hand_contact)

                        else:
                            if stance.prev_move_manip == 0:
                                contact_sequence.append(contact(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.left_leg)),'l_foot'))
                            elif stance.prev_move_manip == 1:
                                contact_sequence.append(contact(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.right_leg)),'r_foot'))
                            elif stance.prev_move_manip == 2:
                                contact_sequence.append(contact(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.left_arm)),'l_hand'))
                            elif stance.prev_move_manip == 3:
                                contact_sequence.append(contact(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.right_arm)),'r_hand'))

                        aligned_stance = copy.deepcopy(segment_path[i])
                        aligned_stance.left_leg = SE3_to_xyzrpy(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.left_leg)))
                        aligned_stance.right_leg = SE3_to_xyzrpy(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.right_leg)))

                        if aligned_stance.left_arm[0] != -99.0:
                            aligned_stance.left_arm = SE3_to_xyzrpy(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.left_arm)))

                        if aligned_stance.right_arm[0] != -99.0:
                            aligned_stance.right_arm = SE3_to_xyzrpy(np.dot(inverse_aligned_coordinate_transform,xyzrpy_to_SE3(stance.right_arm)))

                        if i > 0:
                            aligned_stance.parent = aligned_path[i-1]

                        aligned_path.append(aligned_stance)

                    # update the motion plan library
                    traj_file_path = planning_data_path + 'motion_plan/traj_' + str(len(motion_plan_library)) + '.xml'
                    new_motion_plan = motion_plan(contact_sequence,aligned_path,traj_file_path,segment_waypoint_contact_manips,plan_travel_dist,0)
                    update_motion_plan_library(new_motion_plan,aligned_rave_traj,motion_plan_library,motion_plan_dist)

                    # update the motion plan library for each motion mode
                    segment_motion_mode = segment.motion_mode
                    motion_mode_string = motion_mode_list[segment_motion_mode]
                    motion_plan_library_mm = motion_plan_library_list[segment_motion_mode]
                    motion_plan_dist_mm = motion_plan_dist_list[segment_motion_mode]
                    motion_plan_explore_order_mm = motion_plan_explore_order_list[segment_motion_mode]
                    traj_file_path = planning_data_path + 'motion_plan_' + motion_mode_string + '/traj_' + str(len(motion_plan_library_mm)) + '.xml'
                    new_motion_plan = motion_plan(contact_sequence,aligned_path,traj_file_path,segment_waypoint_contact_manips,plan_travel_dist,0,segment_motion_mode)
                    update_motion_plan_library(new_motion_plan,aligned_rave_traj,motion_plan_library_mm,motion_plan_dist_mm,motion_plan_explore_order_mm,motion_mode=segment_motion_mode)

    IPython.embed()
    return


if __name__ == "__main__":

    contact_sequence_generation_method = 'hybrid'
    path_segmentation_type = 'motion_mode_and_traversability_segmentation'
    traversability_threshold = 0.3
    traversability_threshold_type = 'mean'
    surface_source = 'two_corridor_environment'
    environment_path = 'environment_two_corridor'
    log_file_name = 'exp_result.txt'
    start_env_id = 0
    end_env_id = start_env_id
    recording_data = False
    load_planning_result = False

    i = 1
    while i < len(sys.argv):

        command = sys.argv[i]
        i += 1

        if command == 'contact_sequence_generation_method':
            contact_sequence_generation_method = sys.argv[i]
        elif command == 'path_segmentation_type':
            path_segmentation_type = sys.argv[i]
        elif command == 'traversability_threshold':
            traversability_threshold = float(sys.argv[i])
        elif command == 'surface_source':
            surface_source = sys.argv[i]
        elif command == 'log_file_name':
            log_file_name = sys.argv[i]
        elif command == 'start_env_id':
            start_env_id = int(sys.argv[i])
        elif command == 'end_env_id':
            end_env_id = int(sys.argv[i])
        elif command == 'recording_data':
            if int(sys.argv[i]) == 0:
                recording_data = False
            else:
                recording_data = True
        elif command == 'load_planning_result':
            if int(sys.argv[i]) == 0:
                load_planning_result = False
            else:
                load_planning_result = True
        elif command == 'environment_path':
            environment_path = sys.argv[i]
        elif command == 'traversability_threshold_type':
            if sys.argv[i] == 'mean':
                traversability_threshold_type = 'mean'
            elif sys.argv[i] == 'max':
                traversability_threshold_type = 'max'
            else:
                rave.raveLogInfo('Unknown traversability select criterion: %s. Abort.'%(sys.argv[i]))
                sys.exit()
        else:
            rave.raveLogInfo('Invalid command: %s. Abort.'%(command))
            sys.exit()

        i += 1

    rave.raveLogInfo('Motion Planner Command:')
    rave.raveLogInfo('contact_sequence_generation_method: %s'%(contact_sequence_generation_method))
    rave.raveLogInfo('path_segmentation_type: %s'%(path_segmentation_type))
    rave.raveLogInfo('traversability_threshold_type: %s'%(traversability_threshold_type))
    rave.raveLogInfo('traversability_threshold: %5.2f'%(traversability_threshold))
    rave.raveLogInfo('environment_path: %s'%(environment_path))
    rave.raveLogInfo('surface_source: %s'%(surface_source))
    rave.raveLogInfo('log_file_name: %s'%(log_file_name))
    rave.raveLogInfo('start_env_id: %d'%(start_env_id))
    rave.raveLogInfo('end_env_id: %d'%(end_env_id))
    rave.raveLogInfo('recording_data: %r'%(recording_data))
    rave.raveLogInfo('load_planning_result: %r'%(load_planning_result))

    main(contact_sequence_generation_method = contact_sequence_generation_method,
         path_segmentation_type = path_segmentation_type,
         traversability_threshold_type = traversability_threshold_type,
         traversability_threshold = traversability_threshold,
         environment_path = environment_path,
         surface_source = surface_source,
         log_file_name = log_file_name,
         start_env_id = start_env_id,
         end_env_id = end_env_id,
         recording_data = recording_data,
         load_planning_result = load_planning_result)