#!/usr/bin/env python

from __future__ import print_function

__author__ = 'yu-chi'

# 3rd-Party Imports
import numpy as np
import scipy.spatial as sp
import math
import copy
import time
import pickle
import IPython
import itertools

import openravepy as rave

# Local Imports
from elasticstripspy import ElasticStrips

from config_parameter import *
from transformation_conversion import *
from drawing_functions import *

from step_interpolation import step_interpolation
from contact_space_planner import transition_feasibility, node_IK

from motion_plan import *
from contact_transition import *
from contact_region import *


def show_contact(or_robot,global_contact_sequence):

    contact_kinbodies = [None] * len(global_contact_sequence)
    foot_box_parameter = np.array([[0,0,0,or_robot.foot_h/2,or_robot.foot_w/2,0.005]])
    hand_box_parameter = np.array([[0,0,0,0.005,or_robot.hand_h/2,or_robot.hand_w/2]])

    for i,contact in enumerate(global_contact_sequence):
        contact_kinbodies[i] = or_robot.env.GetKinBody('contact_'+str(i))
        if contact_kinbodies[i] is None:
            contact_kinbodies[i] = rave.RaveCreateKinBody(or_robot.env,'')
            contact_kinbodies[i].SetName('contact_'+str(i))

            if contact.manip == 'l_foot' or contact.manip == 'r_foot':
                contact_kinbodies[i].InitFromBoxes(foot_box_parameter,True)

                if contact.manip == 'l_foot':
                    contact_kinbodies[i].GetLinks()[0].GetGeometries()[0].SetDiffuseColor([203.0/255,27.0/255,69.0/255])
                elif contact.manip == 'r_foot':
                    contact_kinbodies[i].GetLinks()[0].GetGeometries()[0].SetDiffuseColor([34.0/255,125.0/255,81.0/255])

            elif contact.manip == 'l_hand' or contact.manip == 'r_hand':
                contact_kinbodies[i].InitFromBoxes(hand_box_parameter,True)

                if contact.manip == 'l_hand':
                    contact_kinbodies[i].GetLinks()[0].GetGeometries()[0].SetDiffuseColor([17.0/255,50.0/255,133.0/255])
                elif contact.manip == 'r_hand':
                    contact_kinbodies[i].GetLinks()[0].GetGeometries()[0].SetDiffuseColor([255.0/255,196.0/255,8.0/255])

            or_robot.env.AddKinBody(contact_kinbodies[i])

        contact_kinbodies[i].SetTransform(contact.transform)


def delete_contact_visualization(or_robot):
    index = 0
    while True:
        kinbody = or_robot.env.GetKinBody('contact_'+str(index))
        if kinbody != None:
            or_robot.env.Remove(kinbody)
        else:
            break
        index = index + 1


def fitting_env_and_get_plan_cost(or_robot,contact_sequence,contact_regions,q_seq,start,goal,start_bound_radius,goal_bound_radius,dynamic_final_feet_contacts=False):

    start_x = start[0]
    start_y = start[1]
    start_z = start[2]
    start_theta = start[3]

    goal_x = goal[0]
    goal_y = goal[1]
    goal_z = goal[2]
    goal_theta = goal[3]

    seq_transform = xyzrpy_to_SE3([q_seq[0,0],q_seq[1,0],q_seq[2,0],0,0,q_seq[3,0]*rad_to_deg])
    global_contact_sequence = copy.deepcopy(contact_sequence)
    for i in range(len(contact_sequence)):
        global_contact_sequence[i].transform = np.dot(seq_transform,contact_sequence[i].transform)

    goal_array = np.array(goal[0:3])

    # first collect useful information to avoid repetitive calculation.
    r_contact = [None] * len(contact_sequence)
    phi_contact = [None] * len(contact_sequence)
    z_contact = [None] * len(contact_sequence)
    for i,contact in enumerate(contact_sequence):
        r_contact[i] = math.hypot(contact.transform[0,3], contact.transform[1,3])
        phi_contact[i] = math.atan2(contact.transform[1,3],contact.transform[0,3])
        z_contact[i] = contact.transform[2,3]


    start_left_foot_contact_index = -1
    start_right_foot_contact_index = -1

    for i in range(len(global_contact_sequence)):
        if global_contact_sequence[i].manip == 'l_foot' and start_left_foot_contact_index == -1:
            start_left_foot_contact_index = i

        if global_contact_sequence[i].manip == 'r_foot' and start_right_foot_contact_index == -1:
            start_right_foot_contact_index = i

        if start_left_foot_contact_index != -1 and start_right_foot_contact_index != -1:
            break

    final_left_foot_contact_index = -1
    final_right_foot_contact_index = -1

    for i in range(len(global_contact_sequence)-1,-1,-1):
        if global_contact_sequence[i].manip == 'l_foot' and final_left_foot_contact_index == -1:
            final_left_foot_contact_index = i

        if global_contact_sequence[i].manip == 'r_foot' and final_right_foot_contact_index == -1:
            final_right_foot_contact_index = i

        if final_left_foot_contact_index != -1 and final_right_foot_contact_index != -1:
            break


    last_contact_index = len(global_contact_sequence)-1

    if draw_planning_progress:
        show_contact(or_robot,global_contact_sequence)

    # converge the whole contact sequence toward the environment
    epsilon = 0.01
    max_step_size = 0.5
    for k in range(100):
        in_static = False
        # find all velocities(x,y) applied on control points
        q_seq_prev = q_seq.copy()
        seq_transform = xyzrpy_to_SE3([q_seq[0,0],q_seq[1,0],q_seq[2,0],0,0,q_seq[3,0]*rad_to_deg])

        global_contact_sequence = copy.deepcopy(contact_sequence)
        for i in range(len(contact_sequence)):
            global_contact_sequence[i].transform = np.dot(seq_transform,contact_sequence[i].transform)

        if draw_planning_progress:
            delete_contact_visualization(or_robot)
            show_contact(or_robot,global_contact_sequence)

        step = np.zeros((4,1),dtype=float)

        if dynamic_final_feet_contacts:
            shortest_left_foot_to_goal_dist = 9999.0
            shortest_right_foot_to_goal_dist = 9999.0
            for i,contact in enumerate(global_contact_sequence):
                if contact.manip == 'l_foot':
                    tentative_left_foot_to_goal_dist = np.linalg.norm(contact.transform[0:3,3]-goal_array)
                    if tentative_left_foot_to_goal_dist < shortest_left_foot_to_goal_dist:
                        final_left_foot_contact_index = i
                        shortest_left_foot_to_goal_dist = tentative_left_foot_to_goal_dist

                if contact.manip == 'r_foot':
                    tentative_right_foot_to_goal_dist = np.linalg.norm(contact.transform[0:3,3]-goal_array)
                    if tentative_right_foot_to_goal_dist < shortest_right_foot_to_goal_dist:
                        final_right_foot_contact_index = i
                        shortest_right_foot_to_goal_dist = tentative_right_foot_to_goal_dist

            # cropping the motion plan based on the selection of last two foot contacts
            last_contact_index = len(global_contact_sequence)-1
            for i in range(max(final_left_foot_contact_index,final_right_foot_contact_index)+1,len(global_contact_sequence)):
                if contact.manip == 'l_foot' or contact.manip == 'r_foot':
                    last_contact_index = i-1
                    break

        # contact_regions
        while(True):

            J = np.empty((0,4),dtype=float)
            v = np.empty((0,1),dtype=float)

            for i,contact in enumerate(global_contact_sequence[0:last_contact_index+1]):

                vx = 0.0
                vy = 0.0
                vz = 0.0

                if i == start_left_foot_contact_index or i == start_right_foot_contact_index: # starting footstep
                    rs = math.hypot(contact.transform[0,3]-start_x, contact.transform[1,3]-start_y)
                    if rs > start_bound_radius:
                        vx = cs*(start_x-contact.transform[0,3])/start_bound_radius
                        vy = cs*(start_y-contact.transform[1,3])/start_bound_radius
                        vz = cs*(start_z-contact.transform[2,3])
                elif i == final_left_foot_contact_index or i == final_right_foot_contact_index: # goal footstep
                    rg = math.hypot(contact.transform[0,3]-goal_x, contact.transform[1,3]-goal_y)
                    if rg > goal_bound_radius:
                        vx = cg*(goal_x-contact.transform[0,3])/goal_bound_radius
                        vy = cg*(goal_y-contact.transform[1,3])/goal_bound_radius
                        vz = cg*(goal_z-contact.transform[2,3])


                # converge to contact point
                va,_ = attractive_vector(contact,contact_regions)

                vx = vx + va[0,0]
                vy = vy + va[1,0]
                vz = vz + va[2,0]

                if vx != 0.0 or vy != 0.0 or vz != 0.0:
                    Js = np.array([[1,0,0,-r_contact[i]*math.sin(phi_contact[i]+q_seq[3,0])],
                                   [0,1,0,r_contact[i]*math.cos(phi_contact[i]+q_seq[3,0])],
                                   [0,0,1,0]],dtype=float)
                    J = np.concatenate((J,Js),axis=0)
                    vs = np.array([[vx],[vy],[vz]],dtype=float)
                    v = np.concatenate((v,vs),axis=0)

            if v.shape[0] == 0: # converged
                in_static = True
                break

            #calculate pseudo inverse jacobian
            Jplus = np.linalg.pinv(J)

            step = np.dot(Jplus,v)
            step_norm = np.linalg.norm(step)

            if step_norm > max_step_size:
                step = step * (max_step_size/step_norm)

            q_seq = q_seq + step

            inside_limit = True
            if inside_limit:
                break

        step_norm = np.linalg.norm(step)

        if in_static or step_norm < epsilon:
            print('Converged.')
            break

    if draw_planning_progress:
        show_contact(or_robot,global_contact_sequence)

    # raw_input('Final matching.')

    delete_contact_visualization(or_robot)

    if dynamic_final_feet_contacts:
        global_contact_sequence = global_contact_sequence[0:last_contact_index+1]
        contact_sequence = global_contact_sequence

    return contact_sequence_contact_regions_dist(global_contact_sequence,contact_regions)


def plan_env_matching(or_robot,contact_points,contact_regions,
                      dh_grid,start,goal,torso_path,structures,
                      motion_mode,
                      motion_plan_library,
                      motion_plan_clusters,
                      motion_plan_explore_order,
                      planning_time_limit,
                      elasticstrips=None,
                      general_ik_interface=None):

    ######################################################################################
    # Initialization

    rave.raveLogInfo('plan_env_matching')

    DOFNameActiveIndexDict = or_robot.DOFNameActiveIndexDict
    lower_limits = or_robot.lower_limits
    higher_limits = or_robot.higher_limits
    IKInitDOFValues = or_robot.IKInitDOFValues
    StandingPostureDOFValues = or_robot.StandingPostureDOFValues

    or_robot.robot.SetDOFValues(StandingPostureDOFValues)


    goal_x = goal[0]
    goal_y = goal[1]
    goal_z = dh_grid.get_cell_height(dh_grid.position_to_grid_xy((goal_x,goal_y)))
    if goal_z < -0.05:
        goal_z = 0
    goal_theta = goal[2]

    start_x = start[0]
    start_y = start[1]
    start_z = dh_grid.get_cell_height(dh_grid.position_to_grid_xy((start_x,start_y)))
    if start_z < -0.05:
        start_z = 0
    start_theta = start[2]

    goal_dist = math.hypot(goal_x-start_x,goal_y-start_y)

    collision_link_list = ['l_shin','l_foot','r_shin','r_foot','l_forearm','l_palm','r_forearm','r_palm']
    selfcollision_link_pair_list = [('l_foot','r_foot'),('l_shin','r_shin'),
                                    ('l_palm','l_thigh'),('l_index_prox','l_thigh'),('l_index_med','l_thigh'),('l_index_dist','l_thigh'),
                                    ('r_palm','r_thigh'),('r_index_prox','r_thigh'),('r_index_med','r_thigh'),('r_index_dist','r_thigh')]

    i_xpa = DOFNameActiveIndexDict['x_prismatic_joint']
    i_ypa = DOFNameActiveIndexDict['y_prismatic_joint']
    i_zpa = DOFNameActiveIndexDict['z_prismatic_joint']
    i_rra = DOFNameActiveIndexDict['roll_revolute_joint']
    i_pra = DOFNameActiveIndexDict['pitch_revolute_joint']
    i_yra = DOFNameActiveIndexDict['yaw_revolute_joint']

    env = or_robot.env
    start_bound_radius = 0.1
    goal_bound_radius = 0.1

    # DrawRegion(or_robot.env,xyzrpy_to_SE3([0,0,init_z+0.01,0,0,0]),start_bound_radius)
    # DrawRegion(or_robot.env,xyzrpy_to_SE3([goal_x,goal_y,goal_z+0.01,0,0,goal_theta]),goal_bound_radius)

    ######################################################################################

    start_time = time.time()


    solution_found = False

    subsegment_goal = copy.deepcopy(start)

    largest_covered_cell_index = 0
    subsegment_list = []

    # check if every part of the path is covered
    while largest_covered_cell_index != len(torso_path)-1:

        if time.time() - start_time > planning_time_limit:
            break

        subsegment_start = copy.deepcopy(subsegment_goal)
        subsegment_start_to_goal_dist = math.hypot(goal[0]-subsegment_start[0],goal[1]-subsegment_start[1])

        found_matching_motion_plan = False

        for motion_plan_index in motion_plan_explore_order:

            or_robot.robot.SetDOFValues(StandingPostureDOFValues)

            plan = motion_plan_library[motion_plan_index]

            plan.contact_plan[0].parent = None

            plan_shorter_than_torso_path = plan.plan_travel_dist < subsegment_start_to_goal_dist + goal_bound_radius

            # if the plan travel distance is shorter than the subsegment length, use all the motion plan
            if plan_shorter_than_torso_path:

                # the intersection of the motion plan to the closest torso path node
                current_travel_dist = 0
                cell_position = subsegment_start
                subsegment_goal = None
                subsegment_torso_path_range = None
                for i,cell in enumerate(torso_path[largest_covered_cell_index:]):
                    last_cell_position = cell_position
                    cell_position = cell.get_position()
                    last_travel_dist = current_travel_dist
                    current_travel_dist = math.hypot(cell_position[0]-subsegment_start[0],cell_position[1]-subsegment_start[1])

                    if plan.plan_travel_dist < current_travel_dist:
                        # decide subsegment_goal
                        if abs(current_travel_dist-plan.plan_travel_dist) < abs(last_travel_dist-plan.plan_travel_dist):
                            subsegment_goal = cell_position
                        else:
                            subsegment_goal = last_cell_position

                        # decide subsegment_torso_path_range
                        subsegment_torso_path_range = (largest_covered_cell_index,i+largest_covered_cell_index)
                        break

                # means the motion plan cover the whole remaining torso path
                if subsegment_torso_path_range is None:
                    subsegment_goal = cell_position
                    subsegment_torso_path_range = (largest_covered_cell_index,len(torso_path)-1)

                involved_contact_sequence = copy.deepcopy(plan.contact_sequence)

            # if the plan travel distance is longer than the subsegment length, use part of the motion plan
            else:
                # find the contacts that will be involved in the the feature matching
                current_left_foot_contact = None
                current_right_foot_contact = None
                involved_contact_sequence = []
                orientation_offset = None
                final_left_foot_contact = None
                final_right_foot_contact = None

                for contact in plan.contact_sequence:
                    if contact.manip == 'l_foot':
                        current_left_foot_contact = contact
                    elif contact.manip == 'r_foot':
                        current_right_foot_contact = contact

                    if current_left_foot_contact is None or current_right_foot_contact is None:
                        involved_contact_sequence.append(copy.deepcopy(contact))
                        continue

                    current_mean_foot_position = (current_left_foot_contact.transform[0:3,3] + current_right_foot_contact.transform[0:3,3])/2.0

                    current_travel_dist = math.hypot(current_mean_foot_position[0],current_mean_foot_position[1])

                    if current_travel_dist <= subsegment_start_to_goal_dist + goal_bound_radius:
                        involved_contact_sequence.append(copy.deepcopy(contact))
                        orientation_offset = math.atan2(current_mean_foot_position[1],current_mean_foot_position[0])
                        final_left_foot_contact = current_left_foot_contact
                        final_right_foot_contact = current_right_foot_contact
                    else:
                        break

                # decide subsegment_goal
                subsegment_goal = copy.deepcopy(goal)

                # decide subsegment indices range
                subsegment_torso_path_range = (largest_covered_cell_index,len(torso_path)-1)


            subsegment_goal_x = subsegment_goal[0]
            subsegment_goal_y = subsegment_goal[1]
            subsegment_goal_z = dh_grid.get_cell_height(dh_grid.position_to_grid_xy((subsegment_goal_x,subsegment_goal_y)))
            if subsegment_goal_z < -0.05:
                subsegment_goal_z = 0
            subsegment_goal_theta = subsegment_goal[2]

            subsegment_start_x = subsegment_start[0]
            subsegment_start_y = subsegment_start[1]
            subsegment_start_z = dh_grid.get_cell_height(dh_grid.position_to_grid_xy((subsegment_start_x,subsegment_start_y)))
            if subsegment_start_z < -0.05:
                subsegment_start_z = 0
            subsegment_start_theta = subsegment_start[2]

            # initialize the rigid plan pose
            if plan_shorter_than_torso_path:
                init_theta = math.atan2(subsegment_goal_y-subsegment_start_y, subsegment_goal_x-subsegment_start_x)
            else:
                init_theta = math.atan2(subsegment_goal_y-subsegment_start_y, subsegment_goal_x-subsegment_start_x) - orientation_offset

            init_x = subsegment_start_x
            init_y = subsegment_start_y
            init_z = subsegment_start_z

            # the plan are all aligned to the mean position of two feet at the beginning.
            q_seq = np.array([[init_x],[init_y],[init_z],[init_theta]]) # (x,y,z,theta) of the whole sequence

            # jacobian alignment and get the matching score
            plan_cost = fitting_env_and_get_plan_cost(or_robot,involved_contact_sequence,contact_regions,
                                                      q_seq,
                                                      (subsegment_start_x,subsegment_start_y,subsegment_start_z,subsegment_start_theta),
                                                      (subsegment_goal_x,subsegment_goal_y,subsegment_goal_z,subsegment_goal_theta),
                                                      start_bound_radius,
                                                      goal_bound_radius,
                                                      dynamic_final_feet_contacts=True)

            print('Plan %i, Cost: %5.5f'%(motion_plan_index,plan_cost))

            if plan_cost == 9999 or plan_cost == 4999: # length not match
                rave.raveLogWarn('Contact pose distance too high, reject the motion plan.')
                continue

            plan_yaw = q_seq[3,0]*rad_to_deg
            plan_transform = xyzrpy_to_SE3([q_seq[0,0],q_seq[1,0],q_seq[2,0],0,0,plan_yaw])

            plan_contact_sequence = copy.deepcopy(involved_contact_sequence)
            for i in range(len(involved_contact_sequence)):
                plan_contact_sequence[i].transform = np.dot(plan_transform,involved_contact_sequence[i].transform)


            ##################################################
            # initialize the objects needed for elastic strips

            aligned_rave_traj = rave.RaveCreateTrajectory(or_robot.env,'')
            traj_serial = open(plan.traj_path,'r').read()
            aligned_rave_traj.deserialize(traj_serial)


            if plan_shorter_than_torso_path:
                waypoint_contact_manips = copy.deepcopy(plan.waypoint_contact_manips)
                plan_path = copy.deepcopy(plan.contact_plan)

                plan_aligned_traj = [None] * aligned_rave_traj.GetNumWaypoints()
                for i in range(aligned_rave_traj.GetNumWaypoints()):
                    plan_aligned_traj[i] =  copy.deepcopy(aligned_rave_traj.GetWaypoint(i))

            else:
                final_left_foot_contact_position = final_left_foot_contact.transform[0:3,3]
                final_right_foot_contact_position = final_right_foot_contact.transform[0:3,3]

                final_left_foot_contact_reached = False
                final_right_foot_contact_reached = False
                contact_state_bound_indices = None

                for i,node in enumerate(plan.contact_plan):
                    if np.linalg.norm((np.array(node.left_leg[0:3]) - final_left_foot_contact_position)) < 0.002:
                        final_left_foot_contact_reached = True
                    else:
                        if final_left_foot_contact_reached and final_right_foot_contact_reached:
                            contact_state_bound_indices = (0,i-1)
                            break


                    if np.linalg.norm((np.array(node.right_leg[0:3]) - final_right_foot_contact_position)) < 0.002:
                        final_right_foot_contact_reached = True
                    else:
                        if final_left_foot_contact_reached and final_right_foot_contact_reached:
                            contact_state_bound_indices = (0,i-1)
                            break

                    if final_left_foot_contact_reached and final_right_foot_contact_reached and i == len(plan.contact_plan)-1:
                        contact_state_bound_indices = (0,i)

                if contact_state_bound_indices is None:
                    print('empty contact state bound indices')
                    IPython.embed()

                (plan_path,plan_aligned_traj,waypoint_contact_manips) = extract_partial_motion_plan(plan.contact_plan,
                                                                                                    aligned_rave_traj,
                                                                                                    plan.waypoint_contact_manips,
                                                                                                    contact_state_bound_indices)

            if len(waypoint_contact_manips) == 0:
                rave.raveLogWarn('Extracted motion plan does not contain any waypoint, reject the motion plan.')
                continue

            # check if the motion plan orientation matches the start and goal orientation
            if (abs(angle_diff(subsegment_start_theta,plan_path[0].get_virtual_body_yaw()+plan_yaw)) > 30 or
                abs(angle_diff(subsegment_goal_theta,plan_path[-1].get_virtual_body_yaw()+plan_yaw)) > 30):
                rave.raveLogWarn('Path orientation difference too high, reject the motion plan.')
                continue

            ##################################################

            # delete_contact_visualization(or_robot)

            if draw_planning_progress:
                show_contact(or_robot,plan_contact_sequence)

            # DrawLocation(or_robot.env,np.array([[1,0,0,0],[0,1,0,0],[0,0,1,get_z(0,0,structures)+0.01],[0,0,0,1]]),[255,0,0],start_bound_radius)
            # DrawLocation(or_robot.env,np.array([[1,0,0,goal_x],[0,1,0,goal_y],[0,0,1,goal_z+0.01],[0,0,0,1]]),[0,0,255],goal_bound_radius)

            # initialize robot configuration
            or_robot.robot.SetDOFValues(IKInitDOFValues)

            num_waypoints = len(plan_aligned_traj)
            plan_traj = [None] * num_waypoints

            for i in range(num_waypoints):
                q = copy.deepcopy(plan_aligned_traj[i])

                affine_x = q[i_xpa]
                affine_y = q[i_ypa]
                affine_z = q[i_zpa]
                affine_roll = q[i_rra] * rad_to_deg
                affine_pitch = q[i_pra] * rad_to_deg
                affine_yaw = q[i_yra] * rad_to_deg

                affine_transform = xyzrpy_to_SE3([affine_x,affine_y,affine_z,affine_roll,affine_pitch,affine_yaw])

                new_affine_transform = np.dot(plan_transform,affine_transform)

                new_affine_xyzrpy = SE3_to_xyzrpy(new_affine_transform)

                q[i_xpa] = min(max(new_affine_xyzrpy[0],lower_limits[i_xpa]),higher_limits[i_xpa])
                q[i_ypa] = min(max(new_affine_xyzrpy[1],lower_limits[i_ypa]),higher_limits[i_ypa])
                q[i_zpa] = min(max(new_affine_xyzrpy[2],lower_limits[i_zpa]),higher_limits[i_zpa])
                q[i_rra] = min(max(new_affine_xyzrpy[3] * deg_to_rad,lower_limits[i_rra]),higher_limits[i_rra])
                q[i_pra] = min(max(new_affine_xyzrpy[4] * deg_to_rad,lower_limits[i_pra]),higher_limits[i_pra])
                q[i_yra] = min(max(new_affine_xyzrpy[5] * deg_to_rad,lower_limits[i_yra]),higher_limits[i_yra])

                plan_traj[i] = q

            # Elastic Strips
            # trajectory list is going to be a list of lists, each list stores the activeDOF value of the waypoint
            trajectory_list = plan_traj

            manips_list = set(['l_leg','r_leg'])
            for contact_manips in waypoint_contact_manips:
                for contact_manip in contact_manips:
                    if contact_manip[0] == 'l_arm':
                        manips_list.add('l_arm')
                    elif contact_manip[0] == 'r_arm':
                        manips_list.add('r_arm')

            manips_list = list(manips_list)

            support_list = [('l_leg',mu),('r_leg',mu),('l_arm',mu),('r_arm',mu)]

            print('before elastic strips.')

            or_robot.robot.SetDOFValues(StandingPostureDOFValues)
            elasticstrips = ElasticStrips(env, or_robot.robot.GetName())
            or_robot.robot.SetDOFValues(StandingPostureDOFValues)

            print('reachability elastic strips')

            if len(trajectory_list) != len(waypoint_contact_manips):
                rave.raveLogError('Trajectory waypoint number mismatch. Cannot do Elastic Strips.')
                IPython.embed()

            transition_trajectory_list = []
            transition_waypoint_contact_manips = []

            waypoint_manip_num = len(waypoint_contact_manips[0])

            downsampled_trajectory_list = []
            downsampled_waypoint_contact_manips = []

            for i in range(len(trajectory_list)+1):

                if i == len(trajectory_list) or waypoint_manip_num != len(waypoint_contact_manips[i]):

                    if len(transition_trajectory_list) > 2:
                        downsampled_transition_trajectory_list = transition_trajectory_list[0:len(transition_trajectory_list):5]
                        downsampled_transition_waypoint_contact_manips = transition_waypoint_contact_manips[0:len(transition_trajectory_list):5]

                        if (len(transition_trajectory_list)-1) % 5 != 0:
                            downsampled_transition_trajectory_list.append(transition_trajectory_list[-1])
                            downsampled_transition_waypoint_contact_manips.append(transition_waypoint_contact_manips[-1])

                    else:
                        downsampled_transition_trajectory_list = transition_trajectory_list
                        downsampled_transition_waypoint_contact_manips = transition_waypoint_contact_manips

                    downsampled_trajectory_list.extend(downsampled_transition_trajectory_list)
                    downsampled_waypoint_contact_manips.extend(downsampled_transition_waypoint_contact_manips)

                    if i < len(trajectory_list):
                        waypoint_manip_num = len(waypoint_contact_manips[i])
                        transition_trajectory_list = [trajectory_list[i]]
                        transition_waypoint_contact_manips = [waypoint_contact_manips[i]]

                else:
                    transition_trajectory_list.append(trajectory_list[i])
                    transition_waypoint_contact_manips.append(waypoint_contact_manips[i])

            num_waypoints = len(downsampled_trajectory_list)
            trajectory_list = downsampled_trajectory_list
            waypoint_contact_manips = downsampled_waypoint_contact_manips

            with env:
                with or_robot.robot:
                    output_waypoints_list = elasticstrips.RunElasticStrips(manips=manips_list, trajectory=trajectory_list, contact_manips=waypoint_contact_manips, printcommand=False)

            or_robot.robot.SetDOFValues(StandingPostureDOFValues)

            if output_waypoints_list:
                end_time = time.time()

                modified_rave_traj = rave.RaveCreateTrajectory(or_robot.env,'')
                modified_rave_traj.Init(aligned_rave_traj.GetConfigurationSpecification())
                q_modified = [None] * or_robot.robot.GetActiveDOF()

                for w in range(num_waypoints):
                    for i in range(or_robot.robot.GetActiveDOF()):
                        q_modified[i] = output_waypoints_list[w*(or_robot.robot.GetActiveDOF()+1)+i+1]

                    modified_rave_traj.Insert(modified_rave_traj.GetNumWaypoints(),q_modified)

                # extract contact plan path from the trajectory
                prev_contact_manips = waypoint_contact_manips[0]
                or_robot.robot.SetActiveDOFValues(modified_rave_traj.GetWaypoint(0))
                plan_path[0].left_leg = SE3_to_xyzrpy(or_robot.robot.GetManipulator('l_leg').GetTransform())
                plan_path[0].right_leg = SE3_to_xyzrpy(or_robot.robot.GetManipulator('r_leg').GetTransform())

                if plan_path[0].left_arm[0] != -99.0:
                    plan_path[0].left_arm = SE3_to_xyzrpy(or_robot.robot.GetManipulator('l_arm').GetTransform())

                if plan_path[0].right_arm[0] != -99.0:
                    plan_path[0].right_arm = SE3_to_xyzrpy(or_robot.robot.GetManipulator('r_arm').GetTransform())

                plan_path[0].prev_move_manip = None

                current_path_index = 1

                for w in range(1,modified_rave_traj.GetNumWaypoints()):
                    or_robot.robot.SetActiveDOFValues(modified_rave_traj.GetWaypoint(w))

                    if len(prev_contact_manips) < len(waypoint_contact_manips[w]):
                        plan_path[current_path_index].left_leg = copy.copy(plan_path[current_path_index-1].left_leg)
                        plan_path[current_path_index].right_leg = copy.copy(plan_path[current_path_index-1].right_leg)
                        plan_path[current_path_index].left_arm = copy.copy(plan_path[current_path_index-1].left_arm)
                        plan_path[current_path_index].right_arm = copy.copy(plan_path[current_path_index-1].right_arm)

                        if plan_path[current_path_index].prev_move_manip == 0:
                            plan_path[current_path_index].left_leg = SE3_to_xyzrpy(or_robot.robot.GetManipulator('l_leg').GetTransform())
                        elif plan_path[current_path_index].prev_move_manip == 1:
                            plan_path[current_path_index].right_leg = SE3_to_xyzrpy(or_robot.robot.GetManipulator('r_leg').GetTransform())
                        elif plan_path[current_path_index].prev_move_manip == 2:
                            plan_path[current_path_index].left_arm = SE3_to_xyzrpy(or_robot.robot.GetManipulator('l_arm').GetTransform())
                        elif plan_path[current_path_index].prev_move_manip == 3:
                            plan_path[current_path_index].right_arm = SE3_to_xyzrpy(or_robot.robot.GetManipulator('r_arm').GetTransform())

                        while current_path_index < len(plan_path)-1:
                            current_path_index = current_path_index + 1

                            if plan_path[current_path_index].prev_move_manip == 0 or plan_path[current_path_index].prev_move_manip == 1:
                                break
                            elif plan_path[current_path_index].prev_move_manip == 2 and plan_path[current_path_index].left_arm[0] != -99.0:
                                break
                            elif plan_path[current_path_index].prev_move_manip == 3 and plan_path[current_path_index].right_arm[0] != -99.0:
                                break

                    prev_contact_manips = waypoint_contact_manips[w]

                for i in range(len(plan_path)):
                    pose_list = [plan_path[i].left_leg, plan_path[i].right_leg, plan_path[i].left_arm, plan_path[i].right_arm]

                    for j in range(len(pose_list)):
                        pose_list[j][0:3] = [round(v,3) for v in pose_list[j][0:3]]
                        pose_list[j][3:6] = [round(v,1) for v in pose_list[j][3:6]]


                # check if it satisfies the end point balance criterion
                transition_feasible = True
                with env:
                    with or_robot.robot:
                        for plan_node in plan_path:
                            if not transition_feasibility(or_robot,general_ik_interface,plan_node):
                                transition_feasible = False
                                break

                        # check if the iniitial node is feasible
                        if transition_feasible:
                            all_support_list = [('l_leg',mu),('r_leg',mu)]
                            if plan_path[0].left_arm[0] != -99.0:
                                all_support_list.append(('l_arm',mu))

                            if plan_path[0].right_arm[0] != -99.0:
                                all_support_list.append(('r_arm',mu))

                            for support_list in itertools.combinations(all_support_list,len(all_support_list)-1):
                                if node_IK(or_robot,general_ik_interface,plan_path[0],support_list) is None:
                                    transition_feasible = False
                                    break

                        # check if the goal node is feasible
                        if transition_feasible:
                            all_support_list = [('l_leg',mu),('r_leg',mu)]
                            if plan_path[-1].left_arm[0] != -99.0:
                                all_support_list.append(('l_arm',mu))

                            if plan_path[-1].right_arm[0] != -99.0:
                                all_support_list.append(('r_arm',mu))

                            for support_list in itertools.combinations(all_support_list,len(all_support_list)-1):
                                if node_IK(or_robot,general_ik_interface,plan_path[-1],support_list) is None:
                                    transition_feasible = False
                                    break

                if not transition_feasible:
                    rave.raveLogWarn('Transition is not feasible, reject the motion plan.')
                    continue

                found_matching_motion_plan = True

                # final_rave_traj = rave.RaveCreateTrajectory(or_robot.env,'')
                # final_rave_traj.Init(aligned_rave_traj.GetConfigurationSpecification())

                final_rave_traj = None
                final_waypoint_contact_manips = None

                # with env:
                #     (feasible_traj,final_waypoint_contact_manips) = step_interpolation(final_rave_traj,plan_path,or_robot)

                # break
                rave.raveLogInfo('Matching motion plan found.')

                delete_contact_visualization(or_robot)

                subsegment_list.append((plan_path,final_rave_traj,final_waypoint_contact_manips,subsegment_torso_path_range))

                largest_covered_cell_index = subsegment_torso_path_range[1]
                break

            else:
                rave.raveLogWarn('Elastic Strips fail. Not feasible.')

            delete_contact_visualization(or_robot)

        if not found_matching_motion_plan:
            if subsegment_list:
                subsegment_list.append((None,None,None,subsegment_torso_path_range))
            break


    delete_contact_visualization(or_robot)

    if not subsegment_list or time.time() - start_time > planning_time_limit:
        return None
    else:
        return subsegment_list
