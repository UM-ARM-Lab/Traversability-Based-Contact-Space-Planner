import numpy as np
import math
import heapq
import openravepy as rave
import copy
import time
import sys
import IPython

from config_parameter import *
from transformation_conversion import *
from drawing_functions import *


### Verify the simple state feasibility with heuristic bound on end-effector positions.
def state_feasibility_checking(current,or_robot):

    current_left_leg = current.left_leg
    current_right_leg = current.right_leg
    current_left_arm = current.left_arm
    current_right_arm = current.right_arm

    # 1. No place to place foot.
    if current_left_leg[2] == -99.0 or current_right_leg[2] == -99.0:
        return False

    # 2. hand reachability
    current_mean_leg_x = (current_left_leg[0] + current_right_leg[0])/2.0
    current_mean_leg_y = (current_left_leg[1] + current_right_leg[1])/2.0
    current_mean_leg_z = (current_left_leg[2] + current_right_leg[2])/2.0
    current_virtual_body_yaw = current.get_virtual_body_yaw()
    current_leg_mean_transform = xyzrpy_to_SE3([current_mean_leg_x,current_mean_leg_y,current_mean_leg_z,0,0,current_virtual_body_yaw])

    # assuming waist is fixed.
    if current_left_arm[0] != -99.0:
        left_shoulder_position = np.array([[0],[or_robot.shoulder_w/2.0],[or_robot.shoulder_z],[1]],dtype=float)
        left_shoulder_position = np.dot(current_leg_mean_transform,left_shoulder_position)
        left_hand_to_shoulder_dist = math.sqrt((current_left_arm[0]-left_shoulder_position[0,0])**2 +
                                               (current_left_arm[1]-left_shoulder_position[1,0])**2 +
                                               (current_left_arm[2]-left_shoulder_position[2,0])**2)

        if left_hand_to_shoulder_dist > or_robot.max_arm_length or left_hand_to_shoulder_dist < or_robot.min_arm_length:
            return False

    if current_right_arm[0] != -99.0:
        right_shoulder_position = np.array([[0],[-or_robot.shoulder_w/2.0],[or_robot.shoulder_z],[1]],dtype=float)
        right_shoulder_position = np.dot(current_leg_mean_transform,right_shoulder_position)
        right_hand_to_shoulder_dist = math.sqrt((current_right_arm[0]-right_shoulder_position[0,0])**2 +
                                                (current_right_arm[1]-right_shoulder_position[1,0])**2 +
                                                (current_right_arm[2]-right_shoulder_position[2,0])**2)

        if right_hand_to_shoulder_dist > or_robot.max_arm_length or right_hand_to_shoulder_dist < or_robot.min_arm_length:
            return False

    return True


### Inverse kinematics given contact poses of a state(node)
def node_IK(or_robot,general_ik_interface,node,support_list):

    DOFNameActiveIndexDict = or_robot.DOFNameActiveIndexDict
    lower_limits = or_robot.lower_limits
    higher_limits = or_robot.higher_limits
    IKInitDOFValues = or_robot.IKInitDOFValues
    robot = or_robot.robot

    robot.SetDOFValues(IKInitDOFValues)

    maniptm_list = [('l_leg',xyzrpy_to_SE3(node.left_leg)),('r_leg',xyzrpy_to_SE3(node.right_leg))]

    if node.left_arm[0] != -99.0:
        left_arm_transform = xyzrpy_to_SE3(node.left_arm)
        maniptm_list.append(('l_arm',left_arm_transform))

    if node.right_arm[0] != -99.0:
        right_arm_transform = xyzrpy_to_SE3(node.right_arm)
        maniptm_list.append(('r_arm',right_arm_transform))


    ActiveDOFValues = robot.GetActiveDOFValues()
    ActiveDOFValues[DOFNameActiveIndexDict['x_prismatic_joint']] = (node.left_leg[0] + node.right_leg[0])/2.0 # affine x
    ActiveDOFValues[DOFNameActiveIndexDict['y_prismatic_joint']] = (node.left_leg[1] + node.right_leg[1])/2.0 # affine y
    ActiveDOFValues[DOFNameActiveIndexDict['z_prismatic_joint']] = (node.left_leg[2] + node.right_leg[2])/2.0 # affine z
    ActiveDOFValues[DOFNameActiveIndexDict['yaw_revolute_joint']] = node.get_virtual_body_yaw() * math.pi / 180.0 # affine yaw
    robot.SetActiveDOFValues(ActiveDOFValues)

    solved_joint_values = general_ik_interface.DoGeneralIK(execute=False, returnclosest=True, maniptm=maniptm_list, printcommand=False)

    if solved_joint_values is None:
        return None
    else:
        robot.SetActiveDOFValues(solved_joint_values)

    ActiveDOFValues = robot.GetActiveDOFValues()
    for d in range(len(ActiveDOFValues)):
        if ActiveDOFValues[d] <= lower_limits[d] + 0.000001:
            ActiveDOFValues[d] = lower_limits[d] + 0.0001
        elif ActiveDOFValues[d] >= higher_limits[d] - 0.000001:
            ActiveDOFValues[d] = higher_limits[d] - 0.0001
    robot.SetActiveDOFValues(ActiveDOFValues)

    solved_joint_values = general_ik_interface.DoGeneralIK(execute=False, maniptm=maniptm_list, support=support_list, printcommand=False)

    if solved_joint_values is None:
        cogtarg = np.array([0,0,0],dtype=float)
        weight = 0.0
        if ('l_leg',mu) in support_list:
            cogtarg += np.array(node.left_leg[0:3])
            weight += 1.0

        if ('r_leg',mu) in support_list:
            cogtarg += np.array(node.right_leg[0:3])
            weight += 1.0

        cogtarg = (1.0/weight) * cogtarg
        cogtarg[2] += 0.9

        solved_joint_values = general_ik_interface.DoGeneralIK(execute=False, maniptm=maniptm_list, movecog=cogtarg, support=support_list, reusegiwc=True, printcommand=False)

    return solved_joint_values


### Verify the contact transition kinematic feasibility with inverse kinematics
def transition_feasibility(or_robot,general_ik_interface,current):
    prev = current.parent
    if prev is None:
        return True

    support_list = []
    if current.prev_move_manip != 0:
        support_list.append(('l_leg',mu))

    if current.prev_move_manip != 1:
        support_list.append(('r_leg',mu))

    if current.prev_move_manip != 2 and prev.left_arm[0] != -99.0:
        support_list.append(('l_arm',mu))

    if current.prev_move_manip != 3 and prev.right_arm[0] != -99.0:
        support_list.append(('r_arm',mu))

    solved_joint_values = node_IK(or_robot,general_ik_interface,prev,support_list)

    if solved_joint_values is None:
        return False
    else:
        solved_joint_values = node_IK(or_robot,general_ik_interface,current,support_list)

        if solved_joint_values is None:
            return False
        else:
            return True


class node:
    def __init__(self,left_leg,right_leg,left_arm,right_arm,g,h,parent=None,prev_move_manip=None):
        self.left_leg = [None] * len(left_leg)
        self.right_leg = [None] * len(right_leg)
        self.left_arm = [None] * len(left_arm)
        self.right_arm = [None] * len(right_arm)

        self.left_leg[0:3] = copy.copy([round(i,3) for i in left_leg[0:3]])
        self.right_leg[0:3] = copy.copy([round(i,3) for i in right_leg[0:3]])
        self.left_arm[0:3] = copy.copy([round(i,3) for i in left_arm[0:3]])
        self.right_arm[0:3] = copy.copy([round(i,3) for i in right_arm[0:3]])

        self.left_leg[3:6] = copy.copy([round(i,1) for i in left_leg[3:6]])
        self.right_leg[3:6] = copy.copy([round(i,1) for i in right_leg[3:6]])
        self.left_arm[3:6] = copy.copy([round(i,1) for i in left_arm[3:6]])
        self.right_arm[3:6] = copy.copy([round(i,1) for i in right_arm[3:6]])

        self.parent = parent

        self.prev_move_manip = prev_move_manip # previous moving manipulator index
        self.edge_cost = 0.0

        self.g = g
        self.h = h

        self.dr_ability = 1.0

        self.explore_state = 'OPEN'

        self.left_arm_moved = False
        self.right_arm_moved = False

        self.reachability_serial = None
        self.discretized_relative_transform = None

    def __eq__(self, other):
        return self.get_serial() == other.get_serial()

    def __ne__(self, other):
        return not self.__eq__(other)

    def get_serial(self):
        left_leg_serial = ','.join(str(int(round(1000*e))) for e in self.left_leg)
        right_leg_serial = ','.join(str(int(round(1000*e))) for e in self.right_leg)
        left_arm_serial = ','.join(str(int(round(1000*e))) for e in self.left_arm)
        right_arm_serial = ','.join(str(int(round(1000*e))) for e in self.right_arm)

        serial = left_leg_serial + ',' + right_leg_serial + ',' + left_arm_serial + ',' + right_arm_serial

        return serial

    def get_left_horizontal_yaw(self):
        l_leg_rotation = rpy_to_SO3(self.left_leg[3:6])
        cy = l_leg_rotation[0:3,1]
        nx = np.cross(cy,np.array([0,0,1]))
        return (round(math.atan2(nx[1],nx[0]) * rad_to_deg,1))

    def get_right_horizontal_yaw(self):
        r_leg_rotation = rpy_to_SO3(self.right_leg[3:6])
        cy = r_leg_rotation[0:3,1]
        nx = np.cross(cy,np.array([0,0,1]))
        return (round(math.atan2(nx[1],nx[0]) * rad_to_deg,1))

    def get_virtual_body_yaw(self):
        left_horizontal_yaw = self.get_left_horizontal_yaw()
        right_horizontal_yaw = self.get_right_horizontal_yaw()

        return angle_mean(left_horizontal_yaw,right_horizontal_yaw)

    def update_g(self):
        new_g = 0.0
        node = self

        while(node.parent is not None):
            new_g = new_g + node.edge_cost
            node = node.parent

        self.g = new_g

    def at_exact_pose(self, target_pose):
        return (self.left_leg == target_pose[0] and
                self.right_leg == target_pose[1] and
                self.left_arm == target_pose[2] and
                self.right_arm == target_pose[3])

    def get_manip_pose(self, manip_descriptor):
        if isinstance(manip_descriptor, int):
            if manip_descriptor >= len(manip_list):
                rave.raveLogError('Invalid manipulator index: %d'%(manip_descriptor))
                raw_input()

            pose_list = [self.left_leg, self.right_leg, self.left_arm, self.right_arm]
            return pose_list[manip_descriptor]

        elif isinstance(manip_descriptor, basestring):
            if manip_descriptor is 'l_leg':
                return self.left_leg
            elif manip_descriptor is 'r_leg':
                return self.right_leg
            elif manip_descriptor is 'l_arm':
                return self.left_arm
            elif manip_descriptor is 'r_arm':
                return self.right_arm
            else:
                rave.raveLogError('Invalid manipulator name: %s'%(manip_descriptor))
                raw_input()

        rave.raveLogError('Unknown manipulator descriptor type.')

    def get_mean_feet_xyzrpy(self):
        mean_yaw = self.get_virtual_body_yaw()
        mean_x = (self.left_leg[0] + self.right_leg[0])/2.0
        mean_y = (self.left_leg[1] + self.right_leg[1])/2.0
        mean_z = (self.left_leg[2] + self.right_leg[2])/2.0

        return [mean_x,mean_y,mean_z,0,0,mean_yaw]

    def get_mean_feet_pose(self):
        return xyzrpy_to_SE3(self.get_mean_feet_xyzrpy())


### Heuristic estimate given the contact poses of a state
def h_estimation(left_leg,right_leg,left_arm,right_arm,or_robot,torso_pose_grid,motion_mode=0,heuristics='dijkstra',print_cost=False,goal=None,exact_goal_pose=None):

    goal_x = goal[0]
    goal_y = goal[1]
    goal_theta = goal[2]

    exact_goal_pose_diff_manip_num = 0
    making_exact_foot_contact = False
    if exact_goal_pose is not None:
        pose_list = [left_leg, right_leg, left_arm, right_arm]
        for i in range(len(pose_list)):
            if pose_list[i] != exact_goal_pose[i]:
                exact_goal_pose_diff_manip_num += 1
            else:
                if i == 0 or i == 1:
                    making_exact_foot_contact = True


    if heuristics == 'dijkstra':
        mean_x = (left_leg[0] + right_leg[0])/2.0
        mean_y = (left_leg[1] + right_leg[1])/2.0

        l_leg_rotation = rpy_to_SO3(left_leg[3:6])
        l_leg_horizontal_yaw = math.atan2(l_leg_rotation[1,0],l_leg_rotation[0,0]) * rad_to_deg

        r_leg_rotation = rpy_to_SO3(right_leg[3:6])
        r_leg_horizontal_yaw = math.atan2(r_leg_rotation[1,0],r_leg_rotation[0,0]) * rad_to_deg

        mean_theta = (l_leg_horizontal_yaw + r_leg_horizontal_yaw)/2.0

        euclid_dist_to_goal = math.sqrt((goal_x-mean_x)**2 + (goal_y-mean_y)**2)

        (mean_ix,mean_iy,mean_itheta) = torso_pose_grid.position_to_grid((mean_x,mean_y,mean_theta))

        (grid_x,grid_y,grid_theta) = torso_pose_grid.grid_to_position((mean_ix,mean_iy,mean_itheta))

        grid_euclid_dist_to_goal = math.sqrt((goal_x-grid_x)**2 + (goal_y-grid_y)**2)

        grid_to_goal_dist = torso_pose_grid.grid_to_dist((mean_ix,mean_iy,mean_itheta))
        if (grid_to_goal_dist == 4999.0 or grid_to_goal_dist == 9999.0) and making_exact_foot_contact:
            grid_to_goal_dist = 0.0

        if grid_euclid_dist_to_goal != 0 and euclid_dist_to_goal != 0:
            grid_dijkstra_dist_to_goal = grid_to_goal_dist * (euclid_dist_to_goal/grid_euclid_dist_to_goal)
        else:
            grid_dijkstra_dist_to_goal = grid_to_goal_dist

        body_heuristics = grid_dijkstra_dist_to_goal

        if motion_mode == 0 or motion_mode == 1: # all_manipulators and legs_and_left_arm
            if left_arm[0] == -99.0:
                left_hand_dist_to_goal = max(math.sqrt((mean_x-goal_x)**2 + (mean_y-goal_y)**2)-goal_hr,0)
            else:
                left_hand_dist_to_goal = max(math.sqrt((left_arm[0]-goal_x)**2 + (left_arm[1]-goal_y)**2)-goal_hr,0)
        else:
            left_hand_dist_to_goal = 0

        if motion_mode == 0 or motion_mode == 2: # all_manipulators and legs_and_right_arm
            if right_arm[0] == -99.0:
                right_hand_dist_to_goal = max(math.sqrt((mean_x-goal_x)**2 + (mean_y-goal_y)**2)-goal_hr,0)
            else:
                right_hand_dist_to_goal = max(math.sqrt((right_arm[0]-goal_x)**2 + (right_arm[1]-goal_y)**2)-goal_hr,0)
        else:
            right_hand_dist_to_goal = 0

        hand_heuristics = left_hand_dist_to_goal + right_hand_dist_to_goal + dist_step_cost_ratio * (int(math.ceil(left_hand_dist_to_goal/or_robot.max_hand_transition_dist)) + int(math.ceil(right_hand_dist_to_goal/or_robot.max_hand_transition_dist)))

        total_heuristics = body_heuristics + hand_body_cost_ratio * hand_heuristics + 1.0 * exact_goal_pose_diff_manip_num

    elif heuristics == 'euclidean':
        mean_x = (left_leg[0] + right_leg[0])/2.0
        mean_y = (left_leg[1] + right_leg[1])/2.0

        left_euclid_dist_to_goal = math.sqrt((goal_x-left_leg[0])**2 + (goal_y-left_leg[1])**2)
        right_euclid_dist_to_goal = math.sqrt((goal_x-right_leg[0])**2 + (goal_y-right_leg[1])**2)

        body_euclid_dist_to_goal = math.sqrt((goal_x-mean_x)**2 + (goal_y-mean_y)**2)

        l_leg_rotation = rpy_to_SO3(left_leg[3:6])
        l_leg_horizontal_yaw = math.atan2(l_leg_rotation[1,0],l_leg_rotation[0,0]) * rad_to_deg

        r_leg_rotation = rpy_to_SO3(right_leg[3:6])
        r_leg_horizontal_yaw = math.atan2(r_leg_rotation[1,0],r_leg_rotation[0,0]) * rad_to_deg

        mean_theta = (l_leg_horizontal_yaw+r_leg_horizontal_yaw)/2.0

        toward_goal_theta = math.atan2(goal_y-mean_y,goal_x-mean_x) * rad_to_deg

        if body_euclid_dist_to_goal > 0.5:
            body_heuristics = body_euclid_dist_to_goal + dist_angle_cost_ratio*max(abs(toward_goal_theta-mean_theta)-30.0,0) + dist_step_cost_ratio * int(math.ceil(body_euclid_dist_to_goal/or_robot.max_body_transition_dist))
        else:
            body_heuristics = body_euclid_dist_to_goal + dist_angle_cost_ratio*max(abs(goal_theta-mean_theta)-30.0,0) + dist_step_cost_ratio * int(math.ceil(body_euclid_dist_to_goal/or_robot.max_body_transition_dist))


        if motion_mode == 0 or motion_mode == 1: # all_manipulators and legs_and_left_arm
            if left_arm[0] == -99.0:
                left_hand_dist_to_goal = max(math.sqrt((mean_x-goal_x)**2 + (mean_y-goal_y)**2)-goal_hr,0)
            else:
                left_hand_dist_to_goal = max(math.sqrt((left_arm[0]-goal_x)**2 + (left_arm[1]-goal_y)**2)-goal_hr,0)
        else:
            left_hand_dist_to_goal = 0


        if motion_mode == 0 or motion_mode == 2: # all_manipulators and legs_and_right_arm
            if right_arm[0] == -99.0:
                right_hand_dist_to_goal = max(math.sqrt((mean_x-goal_x)**2 + (mean_y-goal_y)**2)-goal_hr,0)
            else:
                right_hand_dist_to_goal = max(math.sqrt((right_arm[0]-goal_x)**2 + (right_arm[1]-goal_y)**2)-goal_hr,0)
        else:
            right_hand_dist_to_goal = 0

        hand_heuristics = left_hand_dist_to_goal + right_hand_dist_to_goal + dist_step_cost_ratio * (int(math.ceil(left_hand_dist_to_goal/or_robot.max_hand_transition_dist)) + int(math.ceil(right_hand_dist_to_goal/or_robot.max_hand_transition_dist)))

        total_heuristics = body_heuristics + hand_body_cost_ratio * hand_heuristics + 1.0 * exact_goal_pose_diff_manip_num

        if print_cost:
            print('body heuristics: %5.3f, hand heuristics: %5.3f'%(body_heuristics,hand_body_cost_ratio * hand_heuristics))

    elif heuristics == 'goal_pose_diff':
        return exact_goal_pose_diff_manip_num

    return total_heuristics


### Project foot contact onto the ground. Return True if there is a projection.
def ground_mapping(or_robot,node,structures_dict,torso_pose_grid,mapping_manip='all'):

    ## Decide which foot we need to project.
    checking_left_foot = False
    checking_right_foot = False

    if mapping_manip is 'all':
        if node.prev_move_manip is not None:
            if node.prev_move_manip == 0: # move left leg
                checking_left_foot = True
            elif node.prev_move_manip == 1: # move right leg
                checking_right_foot = True
        else:
            checking_left_foot = True
            checking_right_foot = True
    elif mapping_manip is 'l_leg':
        checking_left_foot = True
    elif mapping_manip is 'r_leg':
        checking_right_foot = True

    ## Project left foot.
    if checking_left_foot:
        left_foot_projection_prediciton = torso_pose_grid.get_foot_ground_projection((node.left_leg[0],node.left_leg[1]))
        left_safe_footstep = left_foot_projection_prediciton[0]
        projection_surface_id = left_foot_projection_prediciton[1]

        if projection_surface_id is not None:
            struct = structures_dict[projection_surface_id]

            left_foot_projection = struct.projection(or_robot,np.array([[node.left_leg[0]],[node.left_leg[1]],[2.0]]),np.array([[0],[0],[-1]],dtype=float),node.get_left_horizontal_yaw(),'foot',left_safe_footstep)

            if left_foot_projection is not None:
                left_foot_projection_xyzrpy = SE3_to_xyzrpy(left_foot_projection)
                node.left_leg[0:3] = [round(i,3) for i in left_foot_projection_xyzrpy[0:3]]
                node.left_leg[3:6] = [round(i,1) for i in left_foot_projection_xyzrpy[3:6]]
            else:
                return False
        else:
            return False

    ## Project right foot.
    if checking_right_foot:
        right_foot_projection_prediciton = torso_pose_grid.get_foot_ground_projection((node.right_leg[0],node.right_leg[1]))
        right_safe_footstep = right_foot_projection_prediciton[0]
        projection_surface_id = right_foot_projection_prediciton[1]

        if projection_surface_id is not None:
            struct = structures_dict[projection_surface_id]

            right_foot_projection = struct.projection(or_robot,np.array([[node.right_leg[0]],[node.right_leg[1]],[2.0]]),np.array([[0],[0],[-1]],dtype=float),node.get_right_horizontal_yaw(),'foot',right_safe_footstep)

            if right_foot_projection is not None:
                right_foot_projection_xyzrpy = SE3_to_xyzrpy(right_foot_projection)
                node.right_leg[0:3] = [round(i,3) for i in right_foot_projection_xyzrpy[0:3]]
                node.right_leg[3:6] = [round(i,1) for i in right_foot_projection_xyzrpy[3:6]]
            else:
                return False
        else:
            return False

    return True


### Project hand contacts onto the walls. Return True if there is a projection.
def wall_mapping(or_robot,link_no,arm_orientation,node,structures,env=None):

    ## Get the shoulder position, and arm projection vector
    current_mean_leg_x = (node.left_leg[0] + node.right_leg[0])/2.0
    current_mean_leg_y = (node.left_leg[1] + node.right_leg[1])/2.0
    current_mean_leg_z = (node.left_leg[2] + node.right_leg[2])/2.0
    current_virtual_body_yaw = node.get_virtual_body_yaw()

    # assuming waist is fixed.
    if link_no == 2:
        relative_shoulder_position = [0,or_robot.shoulder_w/2.0,or_robot.shoulder_z]
    elif link_no == 3:
        relative_shoulder_position = [0,-or_robot.shoulder_w/2.0,or_robot.shoulder_z]

    current_shoulder_x = current_mean_leg_x + math.cos(current_virtual_body_yaw*deg_to_rad) * relative_shoulder_position[0] - math.sin(current_virtual_body_yaw*deg_to_rad) * relative_shoulder_position[1]
    current_shoulder_y = current_mean_leg_y + math.sin(current_virtual_body_yaw*deg_to_rad) * relative_shoulder_position[0] + math.cos(current_virtual_body_yaw*deg_to_rad) * relative_shoulder_position[1]
    current_shoulder_z = current_mean_leg_z + relative_shoulder_position[2]

    current_shoulder_position = np.array([[current_shoulder_x],
                                          [current_shoulder_y],
                                          [current_shoulder_z],
                                          [1]])

    current_arm_orientation = [0,0]
    if link_no == 2:
        current_arm_orientation[0] = current_virtual_body_yaw + 90.0 - arm_orientation[0]
    elif link_no == 3:
        current_arm_orientation[0] = current_virtual_body_yaw - 90.0 + arm_orientation[0]
    current_arm_orientation[1] = arm_orientation[1]

    ## Find the projected hand contact pose.
    arm_length = 9999.0
    valid_contact = False
    push_direction = 0.0
    arm_pose = [None]*6

    for struct in structures:
        if struct.type == 'others':
            arm_yaw = current_arm_orientation[0] * deg_to_rad
            arm_pitch = current_arm_orientation[1] * deg_to_rad

            arm_ray = np.array([[math.cos(arm_yaw) * math.cos(arm_pitch)],
                                [math.sin(arm_yaw) * math.cos(arm_pitch)],
                                [math.sin(arm_pitch)]])

            if link_no == 2:
                contact_transform = struct.projection(or_robot,current_shoulder_position[0:3,0:1],arm_ray,0.0,'left_hand',False)
            elif link_no == 3:
                contact_transform = struct.projection(or_robot,current_shoulder_position[0:3,0:1],arm_ray,0.0,'right_hand',False)

            if contact_transform is not None: # exist a valid contact on a surface
                temp_arm_length = np.linalg.norm(current_shoulder_position - contact_transform[0:4,3:4])

                if temp_arm_length < arm_length:
                    arm_length = temp_arm_length
                    valid_contact = True
                    arm_pose = SE3_to_xyzrpy(contact_transform)

            else:
                translation = struct.fast_projection_global_frame(current_shoulder_position[0:3,0:1],arm_ray)

                if translation is not None and struct.inside_polygon(translation): # exist projection of center point, but not enough space for a contact
                    temp_arm_length = np.linalg.norm(current_shoulder_position[0:3,0:1] - translation)

                    if temp_arm_length < arm_length:
                        arm_length = temp_arm_length
                        valid_contact = False

    if valid_contact:
        if link_no == 2:
            node.left_arm[0:3] = [round(i,3) for i in arm_pose[0:3]]
            node.left_arm[3:6] = [round(i,1) for i in arm_pose[3:6]]
        elif link_no == 3:
            node.right_arm[0:3] = [round(i,3) for i in arm_pose[0:3]]
            node.right_arm[3:6] = [round(i,1) for i in arm_pose[3:6]]

        return True
    else:
        return False


### Find all branches from the current state.
def branching(current,goal,step_transition_model,hand_transition_model,structures,structures_dict,or_robot,NodeDict,openHeap,G,torso_pose_grid,exact_goal_pose,motion_modes,heuristics='dijkstra',goal_node=None):

    current_left_leg = copy.copy(current.left_leg)
    current_right_leg = copy.copy(current.right_leg)
    current_left_arm = copy.copy(current.left_arm)
    current_right_arm = copy.copy(current.right_arm)

    current_mean_x = (current_left_leg[0] + current_right_leg[0])/2.0
    current_mean_y = (current_left_leg[1] + current_right_leg[1])/2.0
    current_mean_yaw = current.get_virtual_body_yaw()

    l_leg_horizontal_yaw = current.get_left_horizontal_yaw()
    r_leg_horizontal_yaw = current.get_right_horizontal_yaw()

    left_hand_structures = []
    right_hand_structures = []

    for i in range(len(structures)):
        if structures[i].type == 'others':
            left_hand_structures.append(structures[i])
            right_hand_structures.append(structures[i])


    (ix,iy,itheta) = torso_pose_grid.position_to_grid((current_mean_x,current_mean_y,current_mean_yaw))

    branching_factor = 0

    move_manip = [0,1,2,3]

    if current.parent is not None and current.prev_move_manip is not None:
        move_manip.remove(current.prev_move_manip)

    for link_no in move_manip:
        # foot/leg movement

        if link_no == 0 or link_no == 1:
            valid_branching = False

            if 0 in motion_modes and (current_left_arm[0] != -99.0 and current_right_arm[0] != -99.0): # all manipulators
                valid_branching = True or valid_branching

            if 1 in motion_modes and (current_left_arm[0] != -99.0 and current_right_arm[0] == -99.0): # legs_and_left_arm
                valid_branching = True or valid_branching

            if 2 in motion_modes and (current_left_arm[0] == -99.0 and current_right_arm[0] != -99.0): # legs_and_right_arm
                valid_branching = True or valid_branching

            if 3 in motion_modes and (current_left_arm[0] == -99.0 and current_right_arm[0] == -99.0): # legs_only
                valid_branching = True or valid_branching

            if not valid_branching:
                continue

            branching_feet_combination = []

            for step in step_transition_model:
                l_leg_x, l_leg_y,l_leg_z, l_leg_roll, l_leg_pitch, l_leg_yaw = current_left_leg
                r_leg_x, r_leg_y, r_leg_z, r_leg_roll, r_leg_pitch, r_leg_yaw = current_right_leg

                if link_no == 0:
                    l_leg_x = r_leg_x + math.cos(r_leg_horizontal_yaw*(deg_to_rad)) * step[0] - math.sin(r_leg_horizontal_yaw*(deg_to_rad)) * step[1]
                    l_leg_y = r_leg_y + math.sin(r_leg_horizontal_yaw*(deg_to_rad)) * step[0] + math.cos(r_leg_horizontal_yaw*(deg_to_rad)) * step[1]
                    l_leg_z = 99.0
                    l_leg_roll = 0
                    l_leg_pitch = 0
                    l_leg_yaw = r_leg_horizontal_yaw + step[2]

                    l_leg_x = round(l_leg_x,3)
                    l_leg_y = round(l_leg_y,3)
                    l_leg_yaw = round(l_leg_yaw,1)

                elif link_no == 1:
                    r_leg_x = l_leg_x + math.cos(l_leg_horizontal_yaw*(deg_to_rad)) * step[0] - math.sin(l_leg_horizontal_yaw*(deg_to_rad)) * (-step[1])
                    r_leg_y = l_leg_y + math.sin(l_leg_horizontal_yaw*(deg_to_rad)) * step[0] + math.cos(l_leg_horizontal_yaw*(deg_to_rad)) * (-step[1])
                    r_leg_z = 99.0
                    r_leg_roll = 0
                    r_leg_pitch = 0
                    r_leg_yaw = l_leg_horizontal_yaw - step[2]

                    r_leg_x = round(r_leg_x,3)
                    r_leg_y = round(r_leg_y,3)
                    r_leg_yaw = round(r_leg_yaw,1)

                branching_feet_combination.append(([l_leg_x,l_leg_y,l_leg_z,l_leg_roll,l_leg_pitch,l_leg_yaw],
                                                [r_leg_x,r_leg_y,r_leg_z,r_leg_roll,r_leg_pitch,r_leg_yaw],False))

            if exact_goal_pose is not None:

                if link_no == 0 and math.hypot(exact_goal_pose[1][0]-current_right_leg[0],exact_goal_pose[1][1]-current_right_leg[1]) < 0.8:
                    branching_feet_combination.append((exact_goal_pose[0],current_right_leg,True))

                if link_no == 1 and math.hypot(exact_goal_pose[0][0]-current_left_leg[0],exact_goal_pose[0][1]-current_left_leg[1]) < 0.8:
                    branching_feet_combination.append((current_left_leg,exact_goal_pose[1],True))

            for feet_combination in branching_feet_combination:
                l_leg_x, l_leg_y, l_leg_z, l_leg_roll, l_leg_pitch, l_leg_yaw = feet_combination[0]
                r_leg_x, r_leg_y, r_leg_z, r_leg_roll, r_leg_pitch, r_leg_yaw = feet_combination[1]
                branching_to_exact_goal_pose = feet_combination[2]

                new_mean_x = (l_leg_x + r_leg_x)/2
                new_mean_y = (l_leg_y + r_leg_y)/2

                new_left_leg = feet_combination[0]
                new_right_leg = feet_combination[1]

                edge_cost = (math.sqrt((new_mean_x-current_mean_x)**2 + (new_mean_y-current_mean_y)**2) + dist_step_cost_ratio*1)

                if current_left_arm[0] == -99.0 and (motion_modes[0] == 0 or motion_modes[0] == 1): # full_manipulators and legs_and_left_arm
                    edge_cost = edge_cost + hand_body_cost_ratio * (math.sqrt((new_mean_x-current_mean_x)**2 + (new_mean_y-current_mean_y)**2) + dist_step_cost_ratio*1)

                if current_right_arm[0] == -99.0 and (motion_modes[0] == 0 or motion_modes[0] == 2): # fill_manipulators and legs_and_right_arm
                    edge_cost = edge_cost + hand_body_cost_ratio * (math.sqrt((new_mean_x-current_mean_x)**2 + (new_mean_y-current_mean_y)**2) + dist_step_cost_ratio*1)


                if link_no == 0:
                    if (new_left_leg == current_left_leg or
                        new_left_leg[0] >= torso_pose_grid.max_x or
                        new_left_leg[0] < torso_pose_grid.min_x or
                        new_left_leg[1] >= torso_pose_grid.max_y or
                        new_left_leg[1] < torso_pose_grid.min_y):
                        continue
                    child_node = node(new_left_leg,current_right_leg,current_left_arm,current_right_arm,current.g+edge_cost,current.h,current,link_no)
                elif link_no == 1:
                    if (new_right_leg == current_right_leg or
                        new_right_leg[0] >= torso_pose_grid.max_x or
                        new_right_leg[0] < torso_pose_grid.min_x or
                        new_right_leg[1] >= torso_pose_grid.max_y or
                        new_right_leg[1] < torso_pose_grid.min_y):
                        continue
                    child_node = node(current_left_leg,new_right_leg,current_left_arm,current_right_arm,current.g+edge_cost,current.h,current,link_no)

                child_node.edge_cost = edge_cost


                if branching_to_exact_goal_pose or ground_mapping(or_robot,child_node,structures_dict,torso_pose_grid):
                    child_node.h = h_estimation(child_node.left_leg,child_node.right_leg,child_node.left_arm,child_node.right_arm,or_robot,torso_pose_grid,motion_mode=motion_modes[0],heuristics=heuristics,exact_goal_pose=exact_goal_pose,goal=goal)

                    child_serial = child_node.get_serial()

                    temp_node = NodeDict.get(child_serial)

                    if temp_node is not None:
                        if temp_node.explore_state != 'CLOSED':
                            if child_node.g < temp_node.g or (goal_node is not None and temp_node == goal_node):
                                if temp_node.explore_state == 'EXPLORED' or temp_node.explore_state == 'REOPEN':
                                    temp_node.explore_state = 'REOPEN'
                                elif temp_node.explore_state == 'OPEN':
                                    temp_node.explore_state = 'OPEN'
                                temp_node.g = child_node.g
                                temp_node.parent = current
                                temp_node.prev_move_manip = link_no
                                NodeDict[child_serial] = temp_node

                                if temp_node.g + temp_node.h < G:
                                    branching_factor = branching_factor + 1
                                    if temp_node.h != 0:
                                        heapq.heappush(openHeap, (-(G - temp_node.g) / temp_node.h, child_serial))
                                    else:
                                        heapq.heappush(openHeap, (-(G - temp_node.g) / 0.00001, child_serial))

                            if abs(child_node.h-temp_node.h) > 0.001:
                                print('node mismatch, bug!!!1')
                                print(temp_node.get_serial())
                                print(child_node.get_serial())
                                print(temp_node.left_leg)
                                print(temp_node.right_leg)
                                print(temp_node.left_arm)
                                print(temp_node.right_arm)
                                print(child_node.left_leg)
                                print(child_node.right_leg)
                                print(child_node.left_arm)
                                print(child_node.right_arm)
                                print(temp_node.h)
                                print(child_node.h)
                                print(h_estimation(temp_node.left_leg,temp_node.right_leg,temp_node.left_arm,temp_node.right_arm,or_robot,torso_pose_grid,motion_mode=motion_modes[0],heuristics=heuristics,exact_goal_pose=exact_goal_pose,goal=goal))
                                print(h_estimation(child_node.left_leg,child_node.right_leg,child_node.left_arm,child_node.right_arm,or_robot,torso_pose_grid,motion_mode=motion_modes[0],heuristics=heuristics,exact_goal_pose=exact_goal_pose,goal=goal))
                                IPython.embed()
                    else:
                        if child_node.g + child_node.h < G:
                            NodeDict[child_serial] = child_node
                            branching_factor = branching_factor + 1
                            if child_node.h != 0:
                                heapq.heappush(openHeap, (-(G - child_node.g) / child_node.h, child_serial))
                            else:
                                heapq.heappush(openHeap, (-(G - child_node.g) / 0.00001, child_serial))

        # hand/arm movement
        elif link_no == 2 or link_no == 3:

            if link_no == 2:
                hand_structures = left_hand_structures
                if current.left_arm_moved:
                    continue

            elif link_no == 3:
                hand_structures = right_hand_structures
                if current.right_arm_moved:
                    continue


            child_node_list = []

            for arm_orientation in hand_transition_model:

                if exact_goal_pose is None or (current_left_leg != exact_goal_pose[0] and current_right_leg != exact_goal_pose[1]):

                    valid_branching = False

                    if 0 in motion_modes: # allow using all hands in all_manipulator mode
                        valid_branching = True or valid_branching

                    if link_no == 2 and 1 in motion_modes: # allow using left hand in legs_and_left_hand mode
                        valid_branching = True or valid_branching

                    if link_no == 3 and 2 in motion_modes: # allow using right hand in legs_and_right_hand mode
                        valid_branching = True or valid_branching

                    if arm_orientation[0] == -99.0 and arm_orientation[1] == -99.0: # always allow put down hands regardless of motion modes
                        valid_branching = True

                    if not valid_branching:
                        continue

                else:
                    break

                contact_exist = True

                child_node = node(current_left_leg,current_right_leg,current_left_arm,current_right_arm,current.g,current.h,current,link_no)
                child_node.left_arm_moved = current.left_arm_moved
                child_node.right_arm_moved = current.right_arm_moved

                if arm_orientation[0] == -99.0 and arm_orientation[1] == -99.0:
                    if current_left_arm[0] != -99.0 and link_no == 2:
                        new_left_arm = free_floating_pose
                        child_node.left_arm = free_floating_pose
                    elif current_right_arm[0] != -99.0 and link_no == 3:
                        new_right_arm = free_floating_pose
                        child_node.right_arm = free_floating_pose
                    else:
                        continue
                else:
                    contact_exist = wall_mapping(or_robot,link_no,arm_orientation,child_node,hand_structures)
                    child_node.left_arm[0:3] = [round(f,3) for f in child_node.left_arm[0:3]]
                    child_node.left_arm[3:6] = [round(f,1) for f in child_node.left_arm[3:6]]
                    child_node.right_arm[0:3] = [round(f,3) for f in child_node.right_arm[0:3]]
                    child_node.right_arm[3:6] = [round(f,1) for f in child_node.right_arm[3:6]]

                child_serial = child_node.get_serial()

                temp_node = NodeDict.get(child_serial)

                if temp_node is not None and temp_node.explore_state == 'CLOSED':
                    continue

                if contact_exist:
                    child_node_list.append(child_node)

            if exact_goal_pose is not None:
                if (link_no == 2 and (current_left_arm != exact_goal_pose[2])):
                    child_node = node(current_left_leg,current_right_leg,exact_goal_pose[2],current_right_arm,current.g,current.h,current,link_no)
                    child_node.left_arm_moved = current.left_arm_moved
                    child_node.right_arm_moved = current.right_arm_moved
                    child_node_list.append(child_node)


                if (link_no == 3 and (current_right_arm != exact_goal_pose[3])):
                    child_node = node(current_left_leg,current_right_leg,current_left_arm,exact_goal_pose[3],current.g,current.h,current,link_no)
                    child_node.left_arm_moved = current.left_arm_moved
                    child_node.right_arm_moved = current.right_arm_moved
                    child_node_list.append(child_node)


            for child_node in child_node_list:
                child_serial = child_node.get_serial()
                temp_node = NodeDict.get(child_serial)

                new_left_arm = child_node.left_arm
                new_right_arm = child_node.right_arm

                if (link_no == 2 and
                    (new_left_arm == current_left_arm or
                    new_left_arm[0] >= torso_pose_grid.max_x or
                    new_left_arm[0] < torso_pose_grid.min_x or
                    new_left_arm[1] >= torso_pose_grid.max_y or
                    new_left_arm[1] < torso_pose_grid.min_y) and
                    new_left_arm[0] != -99.0):
                    continue

                if (link_no == 3 and
                    (new_right_arm == current_right_arm or
                    new_right_arm[0] >= torso_pose_grid.max_x or
                    new_right_arm[0] < torso_pose_grid.min_x or
                    new_right_arm[1] >= torso_pose_grid.max_y or
                    new_right_arm[1] < torso_pose_grid.min_y) and
                    new_right_arm[0] != -99.0):
                    continue

                # Get the edge cost
                edge_cost = 0

                if link_no == 2:
                    child_node.left_arm_moved = True
                    if new_left_arm[0] == -99.0:
                        edge_cost = math.sqrt((current_mean_x-current_left_arm[0])**2 + (current_mean_y-current_left_arm[1])**2) + dist_step_cost_ratio*1
                    elif current_left_arm[0] == -99.0:
                        edge_cost = math.sqrt((new_left_arm[0]-current_mean_x)**2 + (new_left_arm[1]-current_mean_y)**2) + dist_step_cost_ratio*1
                    else:
                        edge_cost = math.sqrt((new_left_arm[0]-current_left_arm[0])**2 + (new_left_arm[1]-current_left_arm[1])**2) + dist_step_cost_ratio*1

                    if new_left_arm[0] == -99.0 and current_left_arm[0] == -99.0:
                        print('BUG!!!!')
                        IPython.embed()

                elif link_no == 3:
                    child_node.right_arm_moved = True
                    if new_right_arm[0] == -99.0:
                        edge_cost = math.sqrt((current_mean_x-current_right_arm[0])**2 + (current_mean_y-current_right_arm[1])**2) + dist_step_cost_ratio*1
                    elif current_right_arm[0] == -99.0:
                        edge_cost = math.sqrt((new_right_arm[0]-current_mean_x)**2 + (new_right_arm[1]-current_mean_y)**2) + dist_step_cost_ratio*1
                    else:
                        edge_cost = math.sqrt((new_right_arm[0]-current_right_arm[0])**2 + (new_right_arm[1]-current_right_arm[1])**2) + dist_step_cost_ratio*1

                    if new_right_arm[0] == -99.0 and current_right_arm[0] == -99.0:
                        print('BUG!!!!')
                        IPython.embed()

                child_node.g = child_node.g + hand_body_cost_ratio * edge_cost
                child_node.edge_cost = hand_body_cost_ratio * edge_cost
                child_node.h = h_estimation(child_node.left_leg,child_node.right_leg,child_node.left_arm,child_node.right_arm,or_robot,torso_pose_grid,motion_mode=motion_modes[0],heuristics=heuristics,exact_goal_pose=exact_goal_pose,goal=goal)

                if temp_node is not None:
                    if temp_node.explore_state != 'CLOSED':
                        if child_node.g < temp_node.g or (goal_node is not None and temp_node == goal_node):
                            if temp_node.explore_state == 'EXPLORED' or temp_node.explore_state == 'REOPEN':
                                temp_node.explore_state = 'REOPEN'
                            elif temp_node.explore_state == 'OPEN':
                                temp_node.explore_state = 'OPEN'
                            temp_node.g = child_node.g
                            temp_node.parent = current
                            temp_node.prev_move_manip = link_no
                            NodeDict[child_serial] = temp_node
                            if temp_node.g + temp_node.h < G:
                                branching_factor = branching_factor + 1
                                if temp_node.h != 0:
                                    heapq.heappush(openHeap, (-(G - temp_node.g) / temp_node.h, child_serial))
                                else:
                                    heapq.heappush(openHeap, (-(G - temp_node.g) / 0.00001, child_serial))

                        if abs(child_node.h-temp_node.h) > 0.001:
                            print('node mismatch, bug!!!2')
                            print(temp_node.get_serial())
                            print(child_node.get_serial())
                            print(temp_node.left_leg)
                            print(temp_node.right_leg)
                            print(temp_node.left_arm)
                            print(temp_node.right_arm)
                            print(child_node.left_leg)
                            print(child_node.right_leg)
                            print(child_node.left_arm)
                            print(child_node.right_arm)
                            print(temp_node.h)
                            print(child_node.h)
                            print(h_estimation(temp_node.left_leg,temp_node.right_leg,temp_node.left_arm,temp_node.right_arm,or_robot,torso_pose_grid,motion_mode=motion_modes[0],heuristics=heuristics,exact_goal_pose=exact_goal_pose,goal=goal))
                            print(h_estimation(child_node.left_leg,child_node.right_leg,child_node.left_arm,child_node.right_arm,or_robot,torso_pose_grid,motion_mode=motion_modes[0],heuristics=heuristics,exact_goal_pose=exact_goal_pose,goal=goal))
                            IPython.embed()
                else:
                    if child_node.g + child_node.h < G:
                        NodeDict[child_serial] = child_node
                        branching_factor = branching_factor + 1
                        if child_node.h != 0:
                            heapq.heappush(openHeap, (-(G - child_node.g) / child_node.h, child_serial))
                        else:
                            heapq.heappush(openHeap, (-(G - child_node.g) / 0.00001, child_serial))

def retracePath(n):
    path = [n]
    while n.parent is not None:
        n = n.parent
        path.append(n)
    path.reverse()
    return path

def ANA_Star(general_ik_interface,or_robot,structures,torso_pose_grid,goal,initial_node=None,
             step_transition_model=None,hand_transition_model=None,
             motion_mode=0,other_motion_modes=None,heuristics='dijkstra',
             exact_goal_pose=None,goal_radius=goal_br,
             preferred_planning_time=ANA_planning_time,
             planning_time_limit=ANA_time_limit,
             skip_state_feasibility_test=False,
             multiple_motion_modes=False,collect_training_data=False,
             initial_G_h_ratio=999.0):

    env = or_robot.env
    robot = or_robot.robot
    foot_collision_box_1 = or_robot.foot_collision_box_1
    foot_collision_box_2 = or_robot.foot_collision_box_2
    foot_collision_box_3 = or_robot.foot_collision_box_3
    foot_collision_box_4 = or_robot.foot_collision_box_4
    hand_collision_box_1 = or_robot.hand_collision_box_1
    hand_collision_box_2 = or_robot.hand_collision_box_2
    hand_collision_box_3 = or_robot.hand_collision_box_3

    goal_x = goal[0]
    goal_y = goal[1]
    goal_theta = goal[2]

    goal_node = None
    if exact_goal_pose is not None:
        for i in range(len(exact_goal_pose)):
            for j in range(len(exact_goal_pose[i])):
                if j < 3:
                    exact_goal_pose[i][j] = round(exact_goal_pose[i][j],3)
                else:
                    exact_goal_pose[i][j] = round(exact_goal_pose[i][j],1)

        goal_node = node(exact_goal_pose[0],exact_goal_pose[1],exact_goal_pose[2],exact_goal_pose[3],0,0,None,None)

        if exact_goal_pose[2] is not None and exact_goal_pose[3] is None:
            next_motion_mode = 1
        elif exact_goal_pose[2] is None and exact_goal_pose[3] is not None:
            next_motion_mode = 2
        elif exact_goal_pose[2] is not None and exact_goal_pose[3] is not None:
            next_motion_mode = 0
        else:
            next_motion_mode = 3

    global contact_draw_handles

    ANA_starting_time = time.time()
    path = []

    structures_dict = {}
    for struct in structures:
        structures_dict[struct.id] = struct

    initial_node.left_arm_moved = False
    initial_node.right_arm_moved = False

    if collect_training_data:
        G = sys.float_info.max
    else:
        G = initial_node.h * initial_G_h_ratio
    E = G

    start_x = (initial_node.left_leg[0] + initial_node.right_leg[0])/2.0;
    start_y = (initial_node.left_leg[1] + initial_node.right_leg[1])/2.0;

    initial_serial = initial_node.get_serial()
    openHeap = []
    NodeDict = {}

    NodeDict[initial_serial] = initial_node
    if initial_node.h != 0:
        openHeap.append((-(G - initial_node.g) / initial_node.h, initial_serial))
    else:
        openHeap.append((-9999.0, initial_serial))
    heapq.heapify(openHeap)

    explored_node_num = 0

    while openHeap:
        while openHeap:

            if planning_time_limit >= 0 and time.time() - ANA_starting_time > planning_time_limit and not path:
                print('A* Planning Time Out!')
                return []

            if (time.time() - ANA_starting_time > preferred_planning_time or E < 1.1) and path != []:
                path = retracePath(path[len(path)-1])
                DrawStances(path[len(path)-1],or_robot,env,contact_draw_handles)
                return path

            (f_temp,current_serial) = heapq.heappop(openHeap)
            current = NodeDict[current_serial]

            explored_node_num = explored_node_num + 1

            # feasibility checking
            if current.explore_state == 'OPEN' or current.explore_state == 'REOPEN':

                branching_to_goal_pose = False
                if exact_goal_pose is not None:
                    if current.prev_move_manip is not None:
                        current_pose_list = [current.left_leg,current.right_leg,current.left_arm,current.right_arm]
                        manip_id = current.prev_move_manip

                        if current_pose_list[manip_id] == exact_goal_pose[manip_id]:
                            branching_to_goal_pose = True

                feasible = skip_state_feasibility_test or branching_to_goal_pose or current == initial_node or state_feasibility_checking(current, or_robot)

                # collision checking
                if feasible and current != initial_node:
                    collision_box_list = []

                    if current.prev_move_manip == 0:
                        collision_box_1 = foot_collision_box_1
                        collision_box_1.SetTransform(xyzrpy_to_SE3(current.left_leg))
                        collision_box_list.append(collision_box_1)

                    elif current.prev_move_manip == 1:
                        collision_box_1 = foot_collision_box_1
                        collision_box_1.SetTransform(xyzrpy_to_SE3(current.right_leg))
                        collision_box_list.append(collision_box_1)

                    elif current.prev_move_manip == 2 and current.left_arm[0] != -99.0:
                        collision_box_1 = hand_collision_box_1
                        collision_box_1.SetTransform(xyzrpy_to_SE3(current.left_arm))
                        collision_box_list.append(collision_box_1)

                        collision_box_2 = hand_collision_box_2
                        collision_box_2.SetTransform(xyzrpy_to_SE3(current.left_arm))
                        collision_box_list.append(collision_box_2)

                    elif current.prev_move_manip == 3 and current.right_arm[0] != -99.0:
                        collision_box_1 = hand_collision_box_1
                        collision_box_1.SetTransform(xyzrpy_to_SE3(current.right_arm))
                        collision_box_list.append(collision_box_1)

                        collision_box_2 = hand_collision_box_3
                        collision_box_2.SetTransform(xyzrpy_to_SE3(current.right_arm))
                        collision_box_list.append(collision_box_2)

                    if exact_goal_pose is None or not branching_to_goal_pose:
                        for collision_box in collision_box_list:
                            for struct in structures:
                                if env.CheckCollision(collision_box,struct.kinbody):
                                    feasible = False
                                    break

                            if not feasible:
                                break

                    # store the box out of the environment to avoid interfere other collision checking
                    foot_collision_box_1.SetTransform(out_of_env_transform)
                    foot_collision_box_2.SetTransform(out_of_env_transform)
                    foot_collision_box_3.SetTransform(out_of_env_transform)
                    foot_collision_box_4.SetTransform(out_of_env_transform)
                    hand_collision_box_1.SetTransform(out_of_env_transform)
                    hand_collision_box_2.SetTransform(out_of_env_transform)
                    hand_collision_box_3.SetTransform(out_of_env_transform)

                # contact transition kinematic reachability and balance check
                if feasible:
                    with env:
                        with robot:
                            feasible = transition_feasibility(or_robot,general_ik_interface,current)

                if not feasible:
                    if exact_goal_pose is None or current != goal_node:
                        current.explore_state = 'CLOSED'
                        NodeDict[current_serial] = current
                    continue

                current.explore_state = 'EXPLORED'

                if draw_planning_progress:
                    handles = []
                    DrawStances(current,or_robot,env,handles)

                if current.h != 0:
                    if (G-current.g)/current.h < E:
                        E = (G-current.g)/current.h

                current_mean_leg_x = (current.left_leg[0] + current.right_leg[0])/2.0
                current_mean_leg_y = (current.left_leg[1] + current.right_leg[1])/2.0
                body_euclid_dist_to_goal = math.sqrt((goal_x-current_mean_leg_x)**2 + (goal_y-current_mean_leg_y)**2)

                current_virtual_body_yaw = current.get_virtual_body_yaw()
                theta_error = math.acos(math.cos((current_virtual_body_yaw-goal_theta)*deg_to_rad)) * rad_to_deg

                if collect_training_data:
                    goal_reached = (abs(current_mean_leg_x-goal_x) <= torso_pose_grid.resolution/2.0 and
                                    abs(current_mean_leg_y-goal_y) <= torso_pose_grid.resolution/2.0 and
                                    ((motion_mode == 0 and current.left_arm[0] != -99.0 and current.right_arm[0] != -99.0) or
                                     (motion_mode == 1 and current.left_arm[0] != -99.0) or
                                     (motion_mode == 2 and current.right_arm[0] != -99.0) or
                                     motion_mode == 3))
                else:
                    goal_reached = ((exact_goal_pose is None and body_euclid_dist_to_goal <= goal_radius and theta_error <= 30) or
                                    (exact_goal_pose is not None and current.at_exact_pose(exact_goal_pose)))

                if goal_reached:

                    G = current.g

                    handles = []
                    DrawStances(current,or_robot,env,handles)
                    path = retracePath(current)
                    if collect_training_data:
                        print('T = %5.3f, Path found.'%(time.time() - ANA_starting_time))
                    else:
                        print('T = %5.3f, Path found. E: %5.5f, G: %5.5f, # of Steps: %d, # of Nodes Explored: %d'%(time.time() - ANA_starting_time,E,G,len(path),explored_node_num))

                    NodeDict[current_serial] = current

                    break

                # Only branch one step in foot motion when collecting training data
                if not collect_training_data or (current.left_leg == initial_node.left_leg and current.right_leg == initial_node.right_leg):

                    # Determine the motion mode to use
                    if other_motion_modes is None:
                        motion_modes = [motion_mode]
                    else:
                        body_euclid_dist_to_start = math.sqrt((start_x-current_mean_leg_x)**2 + (start_y-current_mean_leg_y)**2)
                        if body_euclid_dist_to_start <= goal_radius:
                            motion_modes = [motion_mode] + other_motion_modes + [0,1,2,3]
                        else:
                            motion_modes = [motion_mode]

                        if multiple_motion_modes:
                            motion_modes = [motion_mode] + other_motion_modes[1:]

                    if exact_goal_pose and body_euclid_dist_to_goal <= 2.0 * goal_radius:
                        motion_modes.append(next_motion_mode)

                    # Find all the branches from the current node
                    branching(current,goal,step_transition_model,hand_transition_model,structures,structures_dict,or_robot,NodeDict,openHeap,G,torso_pose_grid,exact_goal_pose,motion_modes=motion_modes,heuristics=heuristics,goal_node=goal_node)

                NodeDict[current_serial] = current


        # Go through the NodeDict and make nodes' (g+h > G) explored
        openHeap = []
        for key, n in NodeDict.iteritems():
            if n.explore_state == 'OPEN' or n.explore_state == 'REOPEN' or n.explore_state == 'EXPLORED':
                if n.g + n.h >= G:
                    n.explore_state = 'CLOSED'
                    if n.explore_state != NodeDict[key].explore_state:
                        print('explore state mismatch.')
                        raw_input()
                else:
                    if n.explore_state == 'OPEN' or n.explore_state == 'REOPEN':
                        if n.h != 0:
                            openHeap.append((-(G - n.g)/n.h, n.get_serial()))
                        else:
                            openHeap.append((-(G - n.g)/0.00001, n.get_serial()))
        heapq.heapify(openHeap)

    if len(path) != 0:
        path = retracePath(path[len(path)-1])
        DrawStances(path[len(path)-1],or_robot,env,contact_draw_handles)

    return path
