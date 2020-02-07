import numpy as np
import math
import random
import IPython
import itertools
import sys

from config_parameter import *
from transformation_conversion import *


def get_torso_path_segment_traversability(torso_pose_grid, torso_path_segment, motion_type):

    min_traversability = 9999.0
    for i in range(len(torso_path_segment)):
        if i != 0:
            cell1 = torso_path_segment[i-1]
            cell2 = torso_path_segment[i]

            (ix1,iy1,itheta1) = cell1.get_indices()
            (ix2,iy2,itheta2) = cell2.get_indices()

            traversability = torso_pose_grid.env_transition_all_motion_mode_prediction_dict[(ix1,iy1,itheta1,ix2,iy2)][motion_type]

            if traversability < min_traversability:
                min_traversability = traversability

    return min_traversability

def get_torso_path_segment_score(torso_pose_grid,torso_path_segment,motion_mode,traversability_threshold=0.3):

    mean_torso_path_score = get_torso_path_segment_traversability_score_mean(torso_pose_grid,torso_path_segment,motion_mode)

    transition_num = len(torso_path_segment)-1

    score = (mean_torso_path_score-traversability_threshold) * transition_num

    return score

def get_torso_path_segment_traversability_score_step_diff(torso_pose_grid,torso_path_segment,motion_mode,using_library=False):
    traversability_cost_list = [0.0] * (len(torso_path_segment)-1)

    score = 0.0

    for i in range(len(torso_path_segment)):
        if i != 0:
            cell1 = torso_path_segment[i-1]
            cell2 = torso_path_segment[i]

            (ix1,iy1,itheta1) = cell1.get_indices()
            (ix2,iy2,itheta2) = cell2.get_indices()

            if ix1 != ix2 or iy1 != iy2:
                env_transition_predict = torso_pose_grid.env_transition_all_motion_mode_prediction_dict[(ix1,iy1,itheta1,ix2,iy2)][motion_mode]
                env_transition_cost = math.exp(-env_transition_predict)
            else:
                env_transition_cost = 0

            traversability_cost_list[i-1] = env_transition_cost

            if using_library:
                score += (1-env_transition_cost)
            else:
                score += 2.5*env_transition_cost

    return score

def get_torso_path_segment_traversability_score_mean(torso_pose_grid,torso_path_segment,motion_mode):
    traversability_cost_list = [0.0] * (len(torso_path_segment)-1)

    for i in range(len(torso_path_segment)):
        if i != 0:
            cell1 = torso_path_segment[i-1]
            cell2 = torso_path_segment[i]

            (ix1,iy1,itheta1) = cell1.get_indices()
            (ix2,iy2,itheta2) = cell2.get_indices()

            if ix1 != ix2 or iy1 != iy2:
                env_transition_predict = torso_pose_grid.env_transition_all_motion_mode_prediction_dict[(ix1,iy1,itheta1,ix2,iy2)][motion_mode]
                env_transition_cost = math.exp(-env_transition_predict)
            else:
                env_transition_cost = 0

            traversability_cost_list[i-1] = env_transition_cost

    return np.mean(traversability_cost_list)

def get_torso_path_segment_traversability_score_max(torso_pose_grid,torso_path_segment,motion_mode):
    traversability_cost_list = [0.0] * (len(torso_path_segment)-1)

    for i in range(len(torso_path_segment)):
        if i != 0:
            cell1 = torso_path_segment[i-1]
            cell2 = torso_path_segment[i]

            (ix1,iy1,itheta1) = cell1.get_indices()
            (ix2,iy2,itheta2) = cell2.get_indices()

            if ix1 != ix2 or iy1 != iy2:
                env_transition_predict = torso_pose_grid.env_transition_all_motion_mode_prediction_dict[(ix1,iy1,itheta1,ix2,iy2)][motion_mode]
                env_transition_cost = math.exp(-env_transition_predict)
            else:
                env_transition_cost = 0

            traversability_cost_list[i-1] = env_transition_cost

    return np.max(traversability_cost_list)

def get_torso_path_segment_length(torso_pose_grid,torso_path_segment):
    length = 0.0

    for i in range(len(torso_path_segment)):
        if i != len(torso_path_segment)-1:
            cell1 = torso_path_segment[i]
            cell2 = torso_path_segment[i+1]

            (x1,y1,theta1) = cell1.get_position()
            (x2,y2,theta2) = cell2.get_position()

            length += math.hypot(x2-x1,y2-y1)

    return length

def get_segmentation_score(torso_pose_grid,torso_path,segmentation,traversability_threshold=0.3):

    score = 0.0

    for i,segment in enumerate(segmentation):
        start_index = segment[0]
        end_index = segment[1]
        motion_mode = segment[2]

        torso_path_segment = torso_path[start_index:end_index+1]
        segment_score = get_torso_path_segment_score(torso_pose_grid,torso_path_segment,motion_mode,traversability_threshold)


        score += abs(segment_score)

    return round(score,5)

def exhaustive_search_segmentations(torso_pose_grid,torso_path,motion_mode,traversability_threshold=0.3):
    segmentation_list = []

    torso_path_len = len(torso_path)
    max_segment_num = len(torso_path)-1

    min_segment_length = 5

    cut_point_num = torso_path_len-2
    max_select_cut_point_num = int(math.floor(cut_point_num / min_segment_length))

    segmentation_data_points_list = []

    optimal_segmentation = None

    max_segmentation_score = -sys.maxint

    if max_segment_num >= 2*min_segment_length:

        for select_cut_point_num in range(0,max_select_cut_point_num+1):

            for cut_point_bin in itertools.combinations(range(cut_point_num),select_cut_point_num):

                segmentation = []
                segment_start = 0

                segment_too_short = False

                for j in cut_point_bin:
                    segment_end = j+1
                    if (segment_end-segment_start) < min_segment_length or (torso_path_len-segment_end) < min_segment_length:
                        segment_too_short = True
                        break
                    else:
                        segmentation.append((segment_start,segment_end,motion_mode))
                        segment_start = segment_end

                if segment_too_short:
                    continue

                segmentation.append((segment_start,torso_path_len-1,motion_mode))

                segmentation_list.append(segmentation)


                segmentation_score = get_segmentation_score(torso_pose_grid,torso_path,segmentation,traversability_threshold)

                normalized_segmentation_num = float(len(segmentation))/max_segment_num

                segmentation_data_points_list.append((segmentation,normalized_segmentation_num,segmentation_score))

                if segmentation_score > max_segmentation_score:
                    max_segmentation_score = segmentation_score
                    optimal_segmentation = (segmentation,normalized_segmentation_num,segmentation_score)

    else:
        segmentation = [(0,len(torso_path)-1,motion_mode)]
        segmentation_list = [segmentation]

        segmentation_score = get_segmentation_score(torso_pose_grid,torso_path,segmentation,traversability_threshold)
        normalized_segmentation_num = float(len(segmentation))/max_segment_num
        segmentation_data_points_list.append((segmentation,normalized_segmentation_num,segmentation_score))
        optimal_segmentation = (segmentation,normalized_segmentation_num,segmentation_score)


    return optimal_segmentation


def get_torso_path_segmentation(torso_pose_grid,torso_path,path_segmentation_type='motion_mode_and_traversability_segmentation',traversability_threshold=0.3):

    if path_segmentation_type == 'random':
        cut_point_num = len(torso_path)-2
        max_segment_num = len(torso_path)-1

        total_combination_num = 2**(cut_point_num)
        random_cut = random.randint(0,total_combination_num)
        segmentation = []
        cut_point_bin = [int(x) for x in list(bin(random_cut)[2:].zfill(cut_point_num))]
        segment_start = 0

        for k,j in enumerate(cut_point_bin):
            if j == 1:
                segment_end = k+1
                segmentation.append((segment_start,segment_end,random.choice([0,1,2,3])))
                segment_start = segment_end

        segmentation_score = get_segmentation_score(torso_pose_grid,torso_path,segmentation)
        normalized_segmentation_num = float(len(segmentation))/max_segment_num

        path_segmentation = (segmentation,normalized_segmentation_num,segmentation_score)

    elif path_segmentation_type == 'motion_mode_segmentation':
        last_motion_mode = None
        segment_start_index = 0
        path_segment_list = []

        for i in range(len(torso_path)):

            motion_mode = torso_path[i].motion_mode

            if i != 0:
                (ix2,iy2,itheta2) = torso_path[i].get_indices()
                (ix1,iy1,itheta1) = torso_path[i-1].get_indices()

                if motion_mode != last_motion_mode:
                    path_segment_list.append((segment_start_index,i,last_motion_mode))
                    segment_start_index = i

            last_motion_mode = motion_mode

        path_segment_list.append((segment_start_index,i,last_motion_mode))

        path_segmentation = (path_segment_list,1.0,1.0)

    elif path_segmentation_type == 'motion_mode_and_traversability_segmentation':
        last_motion_mode = None
        segment_start_index = 0
        motion_mode_path_segment_list = []

        change_used = False

        for i in range(len(torso_path)):

            motion_mode = torso_path[i].motion_mode

            if i != 0:
                (ix2,iy2,itheta2) = torso_path[i].get_indices()
                (ix1,iy1,itheta1) = torso_path[i-1].get_indices()

                if motion_mode != last_motion_mode:
                    motion_mode_path_segment_list.append((segment_start_index,i,last_motion_mode))
                    segment_start_index = i

            last_motion_mode = motion_mode

        motion_mode_path_segment_list.append((segment_start_index,i,last_motion_mode))

        # segment based on traversability/turning/distance in each motion mode segment
        path_segment_list = []
        for k,motion_mode_segment in enumerate(motion_mode_path_segment_list):
            start_index = motion_mode_segment[0]
            end_index = motion_mode_segment[1]
            motion_mode = motion_mode_segment[2]

            # print('Motion mode segment %d: Index:(%d,%d), Motion mode: %d'%(k,start_index,end_index,motion_mode))

            env_transition_cost_list = []
            for i in range(start_index,end_index+1):
                if i != 0:
                    (ix2,iy2,itheta2) = torso_path[i].get_indices()
                    (ix1,iy1,itheta1) = torso_path[i-1].get_indices()

                    if ix1 != ix2 or iy1 != iy2:
                        env_transition_predict = torso_pose_grid.env_transition_all_motion_mode_prediction_dict[(ix1,iy1,itheta1,ix2,iy2)][motion_mode]
                        env_transition_cost = math.exp(-env_transition_predict)
                    else:
                        env_transition_cost = 0

                    env_transition_cost_list.append(env_transition_cost)

            path_subsegment_list,_,_ = exhaustive_search_segmentations(torso_pose_grid,torso_path[start_index:end_index+1],motion_mode,traversability_threshold)

            for path_subsegment in path_subsegment_list:
                path_segment_list.append((path_subsegment[0]+start_index,path_subsegment[1]+start_index,motion_mode))

        path_segmentation = (path_segment_list,1.0,1.0)

    elif path_segmentation_type == 'specified':
        # path_segmentation = ([(0,int(len(torso_path)/2.0)),(int(len(torso_path)/2.0),len(torso_path)-1)],2.0/(len(torso_path)-1),1.0)
        # path_segmentation = ([(0,len(torso_path)-1,'legs_only')],1.0,1.0)
        path_segmentation = ([(0,int(len(torso_path)/4.0),0),
                              (int(len(torso_path)/4.0),int(2.0*len(torso_path)/4.0),2),
                              (int(2.0*len(torso_path)/4.0),int(3.0*len(torso_path)/4.0),1),
                              (int(3.0*len(torso_path)/4.0),len(torso_path)-1,2)]
                              ,1.0,1.0)

    elif path_segmentation_type == 'no_segmentation':
        path_segmentation = ([(0,len(torso_path)-1,3)],1.0,1.0)

    elif path_segmentation_type == 'all_manipulator_only':
        path_segmentation = ([(0,len(torso_path)-1,2)],1.0,1.0)

    return path_segmentation


