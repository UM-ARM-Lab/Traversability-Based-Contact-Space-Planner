# 3rd-Party Imports
import numpy as np
import math
import openravepy as rave
import copy
import time
import pickle
import IPython

import load_escher

from cbirrtpy import CBiRRT
from traversability_cpp_utility_wrapper import traversability_cpp_utility_wrapper

from contact_space_planner import *
from config_parameter import *
from transformation_conversion import *
from environment_handler import *
from drawing_functions import *
from map_grid import *


def check_manipulator_collision(env,manip_transform,collision_box_list,structures_dict,checking_structures_id=None):

    for i in range(len(collision_box_list)):
        collision_box_list[i].SetTransform(xyzrpy_to_SE3(manip_transform))

        if checking_structures_id is not None and checking_structures_id:
            for struct_id in checking_structures_id:
                struct = structures_dict[struct_id]
                if env.CheckCollision(collision_box_list[i],struct.kinbody):
                    return True
        else:
            for struct_id,struct in structures_dict.iteritems():
                if env.CheckCollision(collision_box_list[i],struct.kinbody):
                    return True

    return False


def get_possible_transitions_and_steps(step_transition_model):

    possible_next_transition_given_torso = {}
    possible_next_step = {}
    for step in step_transition_model: # starting stance
        for manip in leg_manip_list:

            # get the starting stances given the step transition model
            possible_next_transition_given_stance = {}
            if manip is 'l_leg':
                current_left_leg = [-step[0]/2,step[1]/2,0,0,0,step[2]/2]
                current_right_leg = [step[0]/2,-step[1]/2,0,0,0,-step[2]/2]
            elif manip is 'r_leg':
                current_right_leg = [-step[0]/2,-step[1]/2,0,0,0,-step[2]/2]
                current_left_leg = [step[0]/2,step[1]/2,0,0,0,step[2]/2]

            l_leg_x = current_left_leg[0]
            l_leg_y = current_left_leg[1]
            l_leg_z = current_left_leg[2]
            l_leg_roll = 0
            l_leg_pitch = 0
            l_leg_yaw = current_left_leg[5]

            r_leg_x = current_right_leg[0]
            r_leg_y = current_right_leg[1]
            r_leg_z = current_right_leg[2]
            r_leg_roll = 0
            r_leg_pitch = 0
            r_leg_yaw = current_right_leg[5]

            # starting from the starting stance, move one foot contact
            for step2 in step_transition_model: # changing stance
                for manip2 in leg_manip_list: # moving manip
                    if manip2 is 'l_leg':
                        next_l_leg_x = r_leg_x + math.cos(r_leg_yaw*(deg_to_rad)) * step2[0] - math.sin(r_leg_yaw*(deg_to_rad)) * step2[1]
                        next_l_leg_y = r_leg_y + math.sin(r_leg_yaw*(deg_to_rad)) * step2[0] + math.cos(r_leg_yaw*(deg_to_rad)) * step2[1]
                        next_l_leg_yaw = r_leg_yaw + step2[2]
                        next_l_leg_x = round(next_l_leg_x,3)
                        next_l_leg_y = round(next_l_leg_y,3)
                        next_l_leg_yaw = round(next_l_leg_yaw,1)

                        next_left_leg = [next_l_leg_x,next_l_leg_y,0,0,0,next_l_leg_yaw]
                        next_right_leg = current_right_leg
                        moving_leg = next_left_leg

                    elif manip2 is 'r_leg':
                        next_r_leg_x = l_leg_x + math.cos(l_leg_yaw*(deg_to_rad)) * step2[0] - math.sin(l_leg_yaw*(deg_to_rad)) * (-step2[1])
                        next_r_leg_y = l_leg_y + math.sin(l_leg_yaw*(deg_to_rad)) * step2[0] + math.cos(l_leg_yaw*(deg_to_rad)) * (-step2[1])
                        next_r_leg_yaw = l_leg_yaw - step2[2]
                        next_r_leg_x = round(next_r_leg_x,3)
                        next_r_leg_y = round(next_r_leg_y,3)
                        next_r_leg_yaw = round(next_r_leg_yaw,1)

                        next_left_leg = current_left_leg
                        next_right_leg = [next_r_leg_x,next_r_leg_y,0,0,0,next_r_leg_yaw]
                        moving_leg = next_right_leg

                    next_torso = [(next_left_leg[0]+next_right_leg[0])/2.0,(next_left_leg[1]+next_right_leg[1])/2.0,0,0,0,round(angle_mean(next_left_leg[5],next_right_leg[5]),1)]
                    possible_next_transition_given_stance[(manip2, tuple(moving_leg))] = (manip2,copy.copy(moving_leg),next_torso)
                    possible_next_step[tuple(moving_leg)] = None

            stance_key = (l_leg_x,l_leg_y,l_leg_yaw,r_leg_x,r_leg_y,r_leg_yaw)
            possible_next_transition_given_torso[stance_key] = possible_next_transition_given_stance

    return (possible_next_transition_given_torso,possible_next_step)


def get_feasible_step_combinations(or_robot,general_ik_interface,structures,structures_dict,torso_pose_list,torso_pose_grid,step_transition_model,collision_box_list_dict):
    env = or_robot.env
    robot = or_robot.robot

    # get the footstep projection feature
    dummy_node = node(free_floating_pose,free_floating_pose,free_floating_pose,free_floating_pose,0,0)

    transition_result = {}
    feasible_step_combination = []

    leg_manip_list = ['l_leg','r_leg']
    starting_feet_combination_list = set()
    for step in step_transition_model: # starting stance
        for manip in leg_manip_list:
            if manip is 'l_leg':
                current_left_leg = [-step[0]/2,step[1]/2,0,0,0,step[2]/2]
                current_right_leg = [step[0]/2,-step[1]/2,0,0,0,-step[2]/2]
            elif manip is 'r_leg':
                current_right_leg = [-step[0]/2,-step[1]/2,0,0,0,-step[2]/2]
                current_left_leg = [step[0]/2,step[1]/2,0,0,0,step[2]/2]

            starting_feet_combination_list.add((current_left_leg[0],current_left_leg[1],current_left_leg[5],current_right_leg[0],current_right_leg[1],current_right_leg[5]))
    starting_feet_combination_list = list(starting_feet_combination_list)

    left_collision_box_list = collision_box_list_dict['l_leg']
    right_collision_box_list = collision_box_list_dict['r_leg']
    for torso_pose in torso_pose_list:

        torso_x = torso_pose[0]
        torso_y = torso_pose[1]
        torso_yaw = torso_pose[2]

        sty = math.sin(torso_yaw * deg_to_rad) # sin(torso_yaw)
        cty = math.cos(torso_yaw * deg_to_rad) # cos(torso_yaw)

        for stance in starting_feet_combination_list:
            left_leg_x = round(stance[0],3)
            left_leg_y = round(stance[1],3)
            left_leg_yaw = round(stance[2],1)

            right_leg_x = round(stance[3],3)
            right_leg_y = round(stance[4],3)
            right_leg_yaw = round(stance[5],1)

            init_left_leg = [left_leg_x*cty - left_leg_y*sty + torso_x, left_leg_x*sty + left_leg_y*cty + torso_y,0,0,0,torso_yaw+left_leg_yaw]
            init_right_leg = [right_leg_x*cty - right_leg_y*sty + torso_x, right_leg_x*sty + right_leg_y*cty + torso_y,0,0,0,torso_yaw+right_leg_yaw]

            for i in range(6):
                if i < 3:
                    init_left_leg[i] = round(init_left_leg[i],3)
                    init_right_leg[i] = round(init_right_leg[i],3)
                else:
                    init_left_leg[i] = round(init_left_leg[i],1)
                    init_right_leg[i] = round(init_right_leg[i],1)

            dummy_node.left_leg = copy.copy(init_left_leg)
            dummy_node.right_leg = copy.copy(init_right_leg)

            # project on the environment to see its feasibility
            init_stance_feasible = ground_mapping(or_robot,dummy_node,structures_dict,torso_pose_grid)

            # check if the feet are in collision
            if init_stance_feasible:
                left_checking_structures_id = torso_pose_grid.get_neighboring_ground_structure_ids((dummy_node.left_leg[0],dummy_node.left_leg[1]))
                right_checking_structures_id = torso_pose_grid.get_neighboring_ground_structure_ids((dummy_node.right_leg[0],dummy_node.right_leg[1]))

                init_stance_feasible = not check_manipulator_collision(env,dummy_node.left_leg,left_collision_box_list,structures_dict,left_checking_structures_id) and \
                                       not check_manipulator_collision(env,dummy_node.right_leg,right_collision_box_list,structures_dict,right_checking_structures_id)

            if init_stance_feasible:
                feasible_step_combination.append((dummy_node.left_leg,dummy_node.right_leg,init_left_leg,init_right_leg,torso_yaw))

    print('Feasible step combination: %d'%(len(feasible_step_combination)))

    return feasible_step_combination


def main(batch_id=0,mode='all_manipulators',sample_env_num=1000,surface_source='flat_ground_environment'):

    ## initialization of new random environment
    env_handler = environment_handler()
    env = env_handler.env

    ## initialize the robot
    rave.raveLogInfo('Load and Initialize the Robot.')

    or_robot = load_escher.escher(env)

    robot = or_robot.robot
    DOFNameIndexDict = or_robot.DOFNameIndexDict
    DOFNameActiveIndexDict = or_robot.DOFNameActiveIndexDict
    lower_limits = or_robot.lower_limits
    higher_limits = or_robot.higher_limits
    IKInitDOFValues = or_robot.IKInitDOFValues

    resolution = map_grid_resolution

    max_x = 2.0
    min_x = -2.0
    max_y = 2.0
    min_y = -2.0

    # get the transition models
    f = open(planning_data_path + 'step_transition_model_wide_range.txt','r')
    line = ' '
    step_transition_model = []

    while(True):
        line = f.readline()
        if(line == ''):
            break
        step_transition_model.append((float(line[0:5]),float(line[6:11]),float(line[12:17])))
    f.close()

    hand_transition_model = []
    hand_pitch = [-100.0,-90.0,-80.0,-70.0,-60.0,-50.0,-40.0,-30.0,-20.0,-10.0,0.0,10.0,20.0,30.0,40.0,50.0,60.0,70.0,80.0,90.0,100.0]
    hand_yaw = [-20.0,0.0,20.0]
    for pitch in hand_pitch:
        for yaw in hand_yaw:
            hand_transition_model.append((pitch,yaw))


    # construct possible next contact list
    (possible_next_transition_given_torso,possible_next_step) = get_possible_transitions_and_steps(step_transition_model)

    # initialize the IK solver interface
    general_ik_interface = CBiRRT(env, or_robot.robot.GetName())

    # Initialize Escher C++ interface
    traversability_cpp = traversability_cpp_utility_wrapper(env)

    try:
        print('Load footstep_window...')
        out_file = open(planning_data_path + 'footstep_window','r')
        footstep_windows = pickle.load(out_file)
        out_file.close()
        print('Finished loading.')
        # IPython.embed()
    except Exception:
        raw_input('ERROR: Footstep window file not found.')
        return


    # setup collision checking kinbodies
    collision_box_list_dict = {'l_leg':[or_robot.foot_collision_box_1, or_robot.foot_collision_box_2],
                               'r_leg':[or_robot.foot_collision_box_1, or_robot.foot_collision_box_2],
                               'l_arm':[or_robot.hand_collision_box_1, or_robot.hand_collision_box_2],
                               'r_arm':[or_robot.hand_collision_box_1, or_robot.hand_collision_box_3]}

    sample_env_counter = 0

    torso_pose_list = [(0,0,0),(0,0,30),(0,0,60)]
    query_torso_transition = set()
    goal_index_dict = {}
    recording_data = []

    while sample_env_counter < sample_env_num:

        print('Environment Number: %d.'%(sample_env_counter))

        # Initialize the new sampled environment
        env_handler.update_environment(or_robot,surface_source=surface_source)
        structures = env_handler.structures
        structures_dict = {}
        for struct in structures:
            structures_dict[struct.id] = struct

        # Compute the map grid
        torso_pose_grid = map_grid(resolution, structures, or_robot, [min_x,max_x,min_y,max_y])

        goal_num = len(torso_pose_grid.chest_neighbor_window[0]) + len(torso_pose_grid.chest_neighbor_window[30]) + len(torso_pose_grid.chest_neighbor_window[60])

        env_transition_feature_vector = [0] * goal_num

        before_getting_feature_time = time.time()

        feasible_stance = get_feasible_step_combinations(or_robot,general_ik_interface,structures,structures_dict,torso_pose_list,torso_pose_grid,step_transition_model,collision_box_list_dict)


        ####################################################################################################
        # get the mapping from torso pose transition to environment feature: env_transition_feature_vector #
        ####################################################################################################

        if not goal_index_dict:
            index = 0
            for torso_pose in torso_pose_list:
                theta1 = torso_pose[2]
                (ix1,iy1,itheta1) = torso_pose_grid.position_to_grid(torso_pose)
                for neighbor in torso_pose_grid.chest_neighbor_window[theta1]:
                    from_cell = torso_pose_grid.cell_3D_list[ix1][iy1][itheta1]
                    to_cell = torso_pose_grid.cell_3D_list[ix1+neighbor[0]][iy1+neighbor[1]][itheta1]
                    query_torso_transition.add((ix1,iy1,itheta1,ix1+neighbor[0],iy1+neighbor[1]))

                    goal_index_dict[(neighbor[0]*torso_pose_grid.resolution,neighbor[1]*torso_pose_grid.resolution,theta1)] = index
                    index = index + 1
            query_torso_transition = list(query_torso_transition)

            # store goal_index_dict
            print('Storing goal_index_dict...')
            out_file = open('goal_index_dict','w')
            pickle.dump(goal_index_dict,out_file)
            out_file.close()
            print('Storing finished.')


        (footstep_transition_traversability_legs_only,footstep_transition_traverasbility,hand_transition_traversability) = \
        traversability_cpp.SendStartCalculatingTraversabilityCommand(structures,
                                                             footstep_windows,
                                                             footstep_windows,
                                                             query_torso_transition,
                                                             foot_contact_point_resolution,
                                                             torso_pose_grid,
                                                             hand_transition_model)

        index = 0
        for torso_pose in torso_pose_list:
            theta1 = torso_pose[2]
            (ix1,iy1,itheta1) = torso_pose_grid.position_to_grid(torso_pose)
            for neighbor in torso_pose_grid.chest_neighbor_window[theta1]:
                from_cell = torso_pose_grid.cell_3D_list[ix1][iy1][itheta1]
                to_cell = torso_pose_grid.cell_3D_list[ix1+neighbor[0]][iy1+neighbor[1]][itheta1]

                if from_cell.g != 9999.0 and from_cell.g != 4999.0 and to_cell.g != 9999.0 and to_cell.g != 4999.0:
                    hand_traversability = hand_transition_traversability[(ix1,iy1,itheta1)]
                    footstep_traversability = footstep_transition_traverasbility[(ix1,iy1,itheta1,ix1+neighbor[0],iy1+neighbor[1])][0]
                    env_transition_feature_vector[index] = [footstep_traversability,hand_traversability[0],hand_traversability[1],hand_traversability[2],hand_traversability[3]]
                else:
                    env_transition_feature_vector[index] = [0,0,0,0,0]

                index = index + 1

        ####################################################################################################

        after_getting_feature_time = time.time()

        print('Total time for checking ground mapping: %5.5f'%(after_getting_feature_time-before_getting_feature_time))
        successful_goal_count = [0] * goal_num
        # planning_result = [[False]*goal_num] * len(feasible_stance)


        initial_state_projection_numerical_problem = False
        for j,stance in enumerate(feasible_stance):

            print('New Stance No: %d: (%5.3f,%5.3f,%5.1f) (%5.3f,%5.3f,%5.1f)'%(j,stance[2][0],stance[2][1],stance[2][5],stance[3][0],stance[3][1],stance[3][5]))

            # sample the initial stance
            init_left_leg = stance[0]
            init_right_leg = stance[1]
            pre_proj_left_leg = stance[2]
            pre_proj_right_leg = stance[3]
            torso_yaw = stance[4]

            goal_list = torso_pose_grid.chest_neighbor_window[torso_yaw]

            # initialize the node
            init_left_arm = [-99.0,-99.0,-99.0,-99.0,-99.0,-99.0]
            init_right_arm = [-99.0,-99.0,-99.0,-99.0,-99.0,-99.0]

            init_manipulation_objects_configuration = None
            init_manipulating_objects = None

            initial_node = node(init_left_leg,init_right_leg,init_left_arm,init_right_arm,0,0,None,None)

            if not ground_mapping(or_robot,initial_node,structures_dict,torso_pose_grid):
                initial_state_projection_numerical_problem = True
                break

            ## plan footstep to different goal cell to see the result
            for k,goal in enumerate(goal_list):

                goal_x = torso_pose_grid.resolution * goal[0]
                goal_y = torso_pose_grid.resolution * goal[1]
                goal_theta = torso_yaw

                goal_index = goal_index_dict[(goal_x,goal_y,torso_yaw)]
                feature = env_transition_feature_vector[goal_index]

                print('New Goal: %5.3f,%5.3f,%5.3f. Feature: (%5.3f,%5.3f,%5.3f,%5.3f,%5.3f).'%(goal_x,goal_y,goal_theta,feature[0],feature[1],feature[2],feature[3],feature[4]))

                if feature[0] == 0 or (mode != 'legs_only' and feature[1] == 0 and feature[2] == 0 and feature[3] == 0 and feature[4] == 0):
                    continue

                initial_node_copy = copy.deepcopy(initial_node)
                initial_node_copy.h = h_estimation(initial_node_copy.left_leg,
                                                   initial_node_copy.right_leg,
                                                   initial_node_copy.left_arm,
                                                   initial_node_copy.right_arm,
                                                   or_robot,
                                                   torso_pose_grid,
                                                   motion_mode=traversability_regressor_mode_name_index_dict[mode],
                                                   goal=(goal_x,goal_y,goal_theta))
                initial_node_copy.parent = None

                # need to specify motion mode
                path = ANA_Star(general_ik_interface,
                                or_robot,
                                structures,
                                torso_pose_grid,
                                goal=(goal_x,goal_y,goal_theta),
                                initial_node=initial_node_copy,
                                step_transition_model=step_transition_model,
                                hand_transition_model=hand_transition_model,
                                motion_mode=traversability_regressor_mode_name_index_dict[mode],
                                planning_time_limit=-1.0,
                                collect_training_data=True)

                if len(path) != 0:
                    successful_goal_count[goal_index] = successful_goal_count[goal_index] + 1
                    # planning_result[j][goal_index] = True

        if len(feasible_stance) != 0 and not initial_state_projection_numerical_problem:
            # store the data
            recording_data.append((env_transition_feature_vector,successful_goal_count))
            sample_env_counter = sample_env_counter + 1

            if sample_env_counter % 10 == 0 or sample_env_counter == sample_env_num:
                print('Dumping traversability_training_data...')
                out_file = open('traversability_training_data_'+str(batch_id),'w')
                pickle.dump(recording_data,out_file)
                out_file.close()
                print('Dumping finished.')
                IPython.embed()


if __name__ == "__main__":
    batch_id = 0
    mode = 'all_manipulators'
    sample_env_num = 1000

    i = 1
    while i < len(sys.argv):

        command = sys.argv[i]
        i += 1

        if command == 'batch_id':
            batch_id = sys.argv[i]
        elif command == 'mode':
            mode = sys.argv[i]
            if mode not in traversability_regressor_mode_list:
                rave.raveLogInfo('Unknown motion mode: %s. Abort.'%(mode))
                sys.exit()
        elif command == 'sample_env_num':
            sample_env_num = int(sys.argv[i])
        elif command == 'surface_source':
            surface_source = sys.argv[i]
        else:
            rave.raveLogInfo('Invalid command: %s. Abort.'%(command))
            sys.exit()

        i += 1

    main(batch_id,mode,sample_env_num,surface_source)
