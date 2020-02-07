import numpy as np
import math
import openravepy as rave
import time
import pickle
import IPython

import load_escher

from config_parameter import *
from transformation_conversion import *

if __name__ == "__main__":
    env = rave.Environment()  # create openrave environment

    f = open(planning_data_path + 'step_transition_model_wide_range.txt','r')
    line = ' '
    step_transition_model = []
    while(True):
        line = f.readline()
        if(line == ''):
            break
        step_transition_model.append((float(line[0:5]),float(line[6:11]),float(line[12:17])))
    f.close()

    or_robot = load_escher.escher(env)
    leg_manip_list = ['l_leg','r_leg']

    foot_transition_long_radius = or_robot.foot_transition_long_radius
    foot_transition_short_radius = or_robot.foot_transition_short_radius
    chest_transition_long_radius = foot_transition_long_radius/2.0
    chest_transition_short_radius = foot_transition_short_radius/2.0

    resolution = map_grid_resolution
    footstep_window_resolution = foot_contact_point_resolution
    search_window_size = int(math.ceil(foot_transition_long_radius / resolution))

    footstep_windows = {}

    feet_combination_list = set()
    for step in step_transition_model:
        for manip in leg_manip_list:
            if manip is 'l_leg':
                current_left_leg = [-step[0]/2,step[1]/2,0,0,0,step[2]/2]
                current_right_leg = [step[0]/2,-step[1]/2,0,0,0,-step[2]/2]
            elif manip is 'r_leg':
                current_right_leg = [-step[0]/2,-step[1]/2,0,0,0,-step[2]/2]
                current_left_leg = [step[0]/2,step[1]/2,0,0,0,step[2]/2]
            feet_combination_list.add((tuple(current_left_leg),tuple(current_right_leg)))
    feet_combination_list = list(feet_combination_list)

    # Create the footstep windows for 3 orientation: 0, 30, 60 degrees, and the rest can be derived by rotating 90 degrees.
    for it in range(3):

        theta = it*30.0

        sty = math.sin(theta * deg_to_rad)
        cty = math.cos(theta * deg_to_rad)

        inverse_ellipse_frame = np.array([[cty,  sty],
                                          [-sty, cty]])

        temp_chest_neighbor_window = []

        for ix in range(-search_window_size,search_window_size+1):
            for iy in range(-search_window_size,search_window_size+1):
                grid_position = np.array([[ix*resolution],[iy*resolution]])
                transformed_grid_position = np.dot(inverse_ellipse_frame,grid_position)

                c = (transformed_grid_position[0,0]/foot_transition_long_radius)**2 + (transformed_grid_position[1,0]/foot_transition_short_radius)**2

                if c <= 0.25:
                    temp_chest_neighbor_window.append((ix,iy))

        for chest_transition in temp_chest_neighbor_window:
            x = chest_transition[0] * resolution
            y = chest_transition[1] * resolution
            temp_window = []

            for foot_combination in feet_combination_list:
                left_leg_x = round(foot_combination[0][0],3)
                left_leg_y = round(foot_combination[0][1],3)
                left_leg_yaw = round(foot_combination[0][5],1)
                right_leg_x = round(foot_combination[1][0],3)
                right_leg_y = round(foot_combination[1][1],3)
                right_leg_yaw = round(foot_combination[1][5],1)

                current_left_leg = [left_leg_x*cty - left_leg_y*sty, left_leg_x*sty + left_leg_y*cty,0,0,0,theta+left_leg_yaw]
                current_right_leg = [right_leg_x*cty - right_leg_y*sty, right_leg_x*sty + right_leg_y*cty,0,0,0,theta+right_leg_yaw]

                for i in range(6):
                    if i < 3:
                        current_left_leg[i] = round(current_left_leg[i],3)
                        current_right_leg[i] = round(current_right_leg[i],3)
                    else:
                        current_left_leg[i] = round(current_left_leg[i],1)
                        current_right_leg[i] = round(current_right_leg[i],1)

                current_left_leg_transform = xyzrpy_to_SE3(current_left_leg)
                current_right_leg_transform = xyzrpy_to_SE3(current_right_leg)
                legs_key = (current_left_leg[0],current_left_leg[1],current_left_leg[5],current_right_leg[0],current_right_leg[1],current_right_leg[5])

                for step2 in step_transition_model:
                    for manip2 in leg_manip_list:

                        if manip2 is 'l_leg':
                            r_leg_x = current_right_leg[0]
                            r_leg_y = current_right_leg[1]
                            r_leg_yaw = current_right_leg[5]
                            l_leg_x = r_leg_x + math.cos(r_leg_yaw*(deg_to_rad)) * step2[0] - math.sin(r_leg_yaw*(deg_to_rad)) * step2[1]
                            l_leg_y = r_leg_y + math.sin(r_leg_yaw*(deg_to_rad)) * step2[0] + math.cos(r_leg_yaw*(deg_to_rad)) * step2[1]
                            l_leg_yaw = r_leg_yaw + step2[2]
                            l_leg_x = round(l_leg_x,3)
                            l_leg_y = round(l_leg_y,3)
                            l_leg_yaw = round(l_leg_yaw,1)
                            moving_leg_transform = xyzrpy_to_SE3([l_leg_x,l_leg_y,0,0,0,l_leg_yaw])
                        elif manip2 is 'r_leg':
                            l_leg_x = current_left_leg[0]
                            l_leg_y = current_left_leg[1]
                            l_leg_yaw = current_left_leg[5]
                            r_leg_x = l_leg_x + math.cos(l_leg_yaw*(deg_to_rad)) * step2[0] - math.sin(l_leg_yaw*(deg_to_rad)) * (-step2[1])
                            r_leg_y = l_leg_y + math.sin(l_leg_yaw*(deg_to_rad)) * step2[0] + math.cos(l_leg_yaw*(deg_to_rad)) * (-step2[1])
                            r_leg_yaw = l_leg_yaw - step2[2]
                            r_leg_x = round(r_leg_x,3)
                            r_leg_y = round(r_leg_y,3)
                            r_leg_yaw = round(r_leg_yaw,1)
                            moving_leg_transform = xyzrpy_to_SE3([r_leg_x,r_leg_y,0,0,0,r_leg_yaw])

                        mean_x = (l_leg_x + r_leg_x)/2.0
                        mean_y = (l_leg_y + r_leg_y)/2.0

                        # the mean feet position (mean_x,mean_y) is within the cell where (x,y) is at
                        if ((mean_x-x) >= -0.5 * resolution and (mean_x-x) < 0.5 * resolution and
                            (mean_y-y) >= -0.5 * resolution and (mean_y-y) < 0.5 * resolution):

                            moving_x = moving_leg_transform[0,3]
                            moving_y = moving_leg_transform[1,3]

                            moving_footstep_cell = (int(round(moving_x/footstep_window_resolution)),int(round(moving_y/footstep_window_resolution)))
                            left_footstep_cell = (int(round(current_left_leg[0]/footstep_window_resolution)),int(round(current_left_leg[1]/footstep_window_resolution)))
                            right_footstep_cell = (int(round(current_right_leg[0]/footstep_window_resolution)),int(round(current_right_leg[1]/footstep_window_resolution)))

                            temp_window.append((left_footstep_cell,right_footstep_cell,moving_footstep_cell))

            footstep_windows[(chest_transition[0],chest_transition[1],theta)] = temp_window


    out_file = open('footstep_window','w')
    pickle.dump(footstep_windows,out_file)
    out_file.close()

    IPython.embed()