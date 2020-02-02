import numpy as np
import copy

from config_parameter import *
from transformation_conversion import *
from contact_plan_clustering import *

class motion_plan:
    def __init__(self,contact_sequence,contact_plan,traj_path,contact_manips,plan_travel_dist,planning_time,motion_mode=None):
        self.contact_sequence = copy.deepcopy(contact_sequence)
        self.contact_plan = copy.deepcopy(contact_plan)
        self.traj_path = traj_path
        self.waypoint_contact_manips = copy.deepcopy(contact_manips)
        self.plan_travel_dist = plan_travel_dist
        self.planning_time = planning_time
        self.motion_mode = motion_mode
        self.features = None


def trim_down_motion_plan_library(motion_plan_library,num_library,motion_mode,store_file=False):

    trimmed_motion_plan_library = copy.deepcopy(motion_plan_library[0:num_library])

    trimmed_motion_plan_dist = {}
    for pi,plan_i in enumerate(trimmed_motion_plan_library):
        for pj,plan_j in enumerate(trimmed_motion_plan_library[0:pi]):
            dist = plan_to_plan_dist(plan_j,plan_i)
            trimmed_motion_plan_dist[(pj,pi)] = dist

    if store_file:
        if motion_mode is None:
            lib_out_file = open(planning_data_path + 'motion_plan_library','w')
            dist_out_file = open(planning_data_path + 'motion_plan_dist','w')
        else:
            lib_out_file = open(planning_data_path + 'motion_plan_library_' + motion_mode_list[motion_mode],'w')
            dist_out_file = open(planning_data_path + 'motion_plan_dist_' + motion_mode_list[motion_mode],'w')

        pickle.dump(trimmed_motion_plan_library,lib_out_file)
        lib_out_file.close()

        pickle.dump(trimmed_motion_plan_dist,dist_out_file)
        dist_out_file.close()

    trimmed_motion_plan_explore_order = update_motion_plan_explore_order(trimmed_motion_plan_library,motion_mode,store_file)

    return (trimmed_motion_plan_library, trimmed_motion_plan_dist, trimmed_motion_plan_explore_order)


def update_motion_plan_library(new_motion_plan,rave_traj,motion_plan_library,motion_plan_dist,motion_plan_explore_order=None,motion_mode=None):
    # check if this new plan is too close to another one.
    new_motion_plan_dist = {}
    too_close_plan = False
    for pi,plan in enumerate(motion_plan_library):
        dist = plan_to_plan_dist(new_motion_plan,plan)
        new_motion_plan_dist[(pi,len(motion_plan_library))] = dist

        if(dist < min_plan_dist):
            too_close_plan = True
            break

    if(not too_close_plan):
        f_traj_out = open(new_motion_plan.traj_path,'w')
        obj = rave_traj.serialize(0)
        f_traj_out.write(obj)
        f_traj_out.close()

        motion_plan_library.append(new_motion_plan)

        motion_plan_dist.update(new_motion_plan_dist)

        if motion_mode is None:
            lib_out_file = open(planning_data_path + 'motion_plan_library','w')
            dist_out_file = open(planning_data_path + 'motion_plan_dist','w')
        else:
            lib_out_file = open(planning_data_path + 'motion_plan_library_' + motion_mode_list[motion_mode],'w')
            dist_out_file = open(planning_data_path + 'motion_plan_dist_' + motion_mode_list[motion_mode],'w')

        pickle.dump(motion_plan_library,lib_out_file)
        lib_out_file.close()

        pickle.dump(motion_plan_dist,dist_out_file)
        dist_out_file.close()

        if motion_plan_explore_order is not None and motion_mode is not None:
            motion_plan_explore_order = update_motion_plan_explore_order(motion_plan_library,motion_mode)

def update_motion_plan_explore_order(motion_plan_library,motion_mode,store_file=True):
    plan_index_and_travel_dist_list = []

    for j,motion_plan in enumerate(motion_plan_library):
        plan_index_and_travel_dist_list.append((j,motion_plan.plan_travel_dist))

    plan_index_and_travel_dist_list.sort(key=lambda tup: tup[1],reverse=True)
    motion_plan_explore_order = [index_travel_dist_pair[0] for index_travel_dist_pair in plan_index_and_travel_dist_list]

    if store_file:
        out_file = open(planning_data_path + 'motion_plan_explore_order_' + motion_mode_list[motion_mode],'w')
        pickle.dump(motion_plan_explore_order, out_file)
        out_file.close()

    return motion_plan_explore_order
