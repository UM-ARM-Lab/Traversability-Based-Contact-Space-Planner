import math
import openravepy as rave
import copy
import IPython
import sys

from config_parameter import *
from transformation_conversion import *

class contact_transition:
    def __init__(self,traj,waypoint_contact_manips,start_contact_state,goal_contact_state):
        self.traj = traj
        self.waypoint_contact_manips = waypoint_contact_manips
        self.manip_name = manip_list[goal_contact_state.prev_move_manip]
        self.from_pose = start_contact_state.get_manip_pose(self.manip_name)
        self.to_pose = goal_contact_state.get_manip_pose(self.manip_name)
        self.transition_dist = math.sqrt((self.to_pose[0]-self.from_pose[0])**2
                                       + (self.to_pose[1]-self.from_pose[1])**2
                                       + (self.to_pose[2]-self.from_pose[2])**2)


def reindex_waypoint_contact_manips(waypoint_contact_manips):

    manip_group_mapping = {}
    manip_group_counter = 0
    prev_contact_manips_set = []

    new_waypoint_contact_manips = []
    for contact_manip_group in waypoint_contact_manips:
        contact_manips = []
        contact_manips_set = [contact_manip[0] for contact_manip in contact_manip_group]
        for contact_manip in contact_manip_group:
            contact_manip_name = contact_manip[0]

            if contact_manip_name not in prev_contact_manips_set:
                manip_group_mapping[contact_manip_name] = manip_group_counter
                manip_group_counter += 1

            contact_manips.append((contact_manip_name,manip_group_mapping[contact_manip_name]))

        new_waypoint_contact_manips.append(contact_manips)
        prev_contact_manips_set = contact_manips_set

    return new_waypoint_contact_manips


def get_diff_manip(waypoint_contact_manip1, waypoint_contact_manip2):
    # extract the diff manipulator between two waypoints
    if len(waypoint_contact_manip1) < len(waypoint_contact_manip2):
        shorter_manipulator_list = [manip[0] for manip in waypoint_contact_manip1]
        longer_manipulator_list = [manip[0] for manip in waypoint_contact_manip2]
    else:
        shorter_manipulator_list = [manip[0] for manip in waypoint_contact_manip2]
        longer_manipulator_list = [manip[0] for manip in waypoint_contact_manip1]

    for key,manip in manip_list.iteritems():
        if(manip in longer_manipulator_list and manip not in shorter_manipulator_list):
            return manip

    return None


def extract_partial_motion_plan(contact_state_path,rave_traj,waypoint_contact_manips,contact_state_bound_indices):

    start_contact_state_index = contact_state_bound_indices[0]
    end_contact_state_index = contact_state_bound_indices[1]

    prev_cut_waypoint_index = 0
    cut_waypoint_index = 0

    last_contact_change_index = 0
    current_contact_change_index = 0

    contact_transition_traj = []
    contact_transition_waypoint_contact_manips = []

    last_change = None
    current_change = None

    last_move_manip = None
    current_move_manip = None

    current_contact_state_index = 0
    partial_contact_state_path = copy.deepcopy(contact_state_path[start_contact_state_index:end_contact_state_index+1])
    partial_traj = []
    partial_waypoint_contact_manips = []

    for waypoint_index in range(len(waypoint_contact_manips)):
        if (waypoint_index != 0 and
            len(waypoint_contact_manips[waypoint_index]) != len(waypoint_contact_manips[waypoint_index-1])):
            current_contact_change_index = waypoint_index

            if len(waypoint_contact_manips[waypoint_index]) < len(waypoint_contact_manips[waypoint_index-1]):
                current_change = 'decrease'
            elif len(waypoint_contact_manips[waypoint_index]) > len(waypoint_contact_manips[waypoint_index-1]):
                current_change = 'increase'

            current_move_manip = get_diff_manip(waypoint_contact_manips[waypoint_index-1], waypoint_contact_manips[waypoint_index])

            if((last_change == 'increase' and current_change == 'decrease') or
               (last_change == 'increase' and current_change == 'increase') or
               (last_change == 'decrease' and current_change == 'decrease') or
               (last_change == 'decrease' and current_change == 'increase' and last_move_manip != current_move_manip)):
                cut_waypoint_index = int((current_contact_change_index + (last_contact_change_index-1))/2)+1

                if current_contact_state_index >= start_contact_state_index and current_contact_state_index < end_contact_state_index:
                    partial_traj += [rave_traj.GetWaypoint(k) for k in range(prev_cut_waypoint_index,cut_waypoint_index)]
                    partial_waypoint_contact_manips += waypoint_contact_manips[prev_cut_waypoint_index:cut_waypoint_index]

                prev_cut_waypoint_index = cut_waypoint_index
                current_contact_state_index += 1

                if current_contact_state_index >= end_contact_state_index:
                    break

            last_change = copy.copy(current_change)
            last_move_manip = copy.copy(current_move_manip)
            last_contact_change_index = current_contact_change_index


        if waypoint_index == len(waypoint_contact_manips)-1:
            cut_waypoint_index = waypoint_index+1

            if current_contact_state_index >= start_contact_state_index and current_contact_state_index < end_contact_state_index:
                partial_traj += [rave_traj.GetWaypoint(k) for k in range(prev_cut_waypoint_index,cut_waypoint_index)]
                partial_waypoint_contact_manips += waypoint_contact_manips[prev_cut_waypoint_index:cut_waypoint_index]


    partial_waypoint_contact_manips = reindex_waypoint_contact_manips(partial_waypoint_contact_manips)

    return (partial_contact_state_path,partial_traj,partial_waypoint_contact_manips)
