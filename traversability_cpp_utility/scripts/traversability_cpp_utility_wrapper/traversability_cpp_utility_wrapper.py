# Python interface for the C++ Escher motion planning module
import openravepy as rave

import time
import sys
import numpy as np
import scipy

class traversability_cpp_utility_wrapper(object):
    """
    Wrapper class for the OpenRave Escher plugin.

    :type env rave.Environment
    :type problem rave.Problem
    :type manip_indices dict[rave.Manipulator|str, int]
    """

    def __init__(self, env):

        self.module = rave.RaveCreateModule(env,'TraversabilityCPPUtility')
        env.AddModule(self.module,'')
        self.env = env

    # OpenRave C++ plugin is called by sending string command. We can add parameters in this function to construct the command, and decode in C++ side.
    # For example, I can add an option whether to turn on the parallelization or not

    def SendStartCalculatingTraversabilityCommand(self, structures=None, footstep_windows_legs_only=None, footstep_windows=None, torso_transitions=None, footstep_window_grid_resolution=None,
                                                torso_pose_grid=None, hand_transition_model=None, parallelization=None, printing=False):
        start = time.time()

        cmd = ['StartCalculatingTraversability']

        if printing:
            cmd.append('printing')

        if structures is not None:
            cmd.append('structures')
            cmd.append(len(structures))

            for struct in structures:
                cmd.append(struct.geometry)
                cmd.append(struct.kinbody.GetName())
                cmd.append(struct.id)

                # plane parameters
                cmd.extend((struct.nx,struct.ny,struct.nz,struct.c))

                # vertices
                cmd.append(len(struct.vertices))

                for vertex in struct.vertices:
                    cmd.extend(vertex)

                # boundaries
                cmd.append(len(struct.boundaries))

                for edge in struct.boundaries:
                    cmd.extend(edge)

                # trimesh types
                cmd.append(struct.type)


        if footstep_windows_legs_only is not None:
            cmd.append('transition_footstep_window_cells_legs_only')
            cmd.append(len(footstep_windows_legs_only))

            for key,footstep_window in footstep_windows_legs_only.iteritems():
                cmd.append(key[0])
                cmd.append(key[1])
                cmd.append(int(key[2]))
                cmd.append(len(footstep_window))

                for cell_tuple in footstep_window:
                    cmd.extend(cell_tuple[0])
                    cmd.extend(cell_tuple[1])
                    cmd.extend(cell_tuple[2])

        if footstep_windows is not None:
            cmd.append('transition_footstep_window_cells')
            cmd.append(len(footstep_windows))

            for key,footstep_window in footstep_windows.iteritems():
                cmd.append(key[0])
                cmd.append(key[1])
                cmd.append(int(key[2]))
                cmd.append(len(footstep_window))

                for cell_tuple in footstep_window:
                    cmd.extend(cell_tuple[0])
                    cmd.extend(cell_tuple[1])
                    cmd.extend(cell_tuple[2])

        if torso_transitions is not None:
            cmd.append('torso_transitions')
            cmd.append(len(torso_transitions))

            for transition in torso_transitions:
                cmd.extend(transition)

        if footstep_window_grid_resolution is not None:
            cmd.append('footstep_window_grid_resolution')
            cmd.append(footstep_window_grid_resolution)

        if torso_pose_grid is not None:
            cmd.append('map_grid')
            cmd.append(torso_pose_grid.min_x)
            cmd.append(torso_pose_grid.max_x)
            cmd.append(torso_pose_grid.min_y)
            cmd.append(torso_pose_grid.max_y)
            cmd.append(torso_pose_grid.resolution)

            for i in range(torso_pose_grid.dim_x):
                for j in range(torso_pose_grid.dim_y):

                    cmd.append(torso_pose_grid.cell_2D_list[i][j].height)

                    if torso_pose_grid.cell_2D_list[i][j].foot_ground_projection[0]:
                        cmd.append(1)
                    else:
                        cmd.append(0)

                    if torso_pose_grid.cell_2D_list[i][j].foot_ground_projection[1] is None:
                        cmd.append(-99)
                    else:
                        cmd.append(torso_pose_grid.cell_2D_list[i][j].foot_ground_projection[1])

                    cmd.append(len(torso_pose_grid.cell_2D_list[i][j].all_ground_structures))
                    cmd.extend(torso_pose_grid.cell_2D_list[i][j].all_ground_structures)

                    for k in range(torso_pose_grid.dim_theta):
                        if torso_pose_grid.cell_3D_list[i][j][k].parent is not None:
                            cmd.extend(torso_pose_grid.cell_3D_list[i][j][k].parent.get_indices())
                        else:
                            cmd.extend((-99,-99,-99))
                        cmd.append(torso_pose_grid.cell_3D_list[i][j][k].g)
                        cmd.append(torso_pose_grid.cell_3D_list[i][j][k].h)

                        cmd.append(len(torso_pose_grid.cell_3D_list[i][j][k].left_hand_checking_surface_index))
                        cmd.extend(torso_pose_grid.cell_3D_list[i][j][k].left_hand_checking_surface_index)
                        cmd.append(len(torso_pose_grid.cell_3D_list[i][j][k].right_hand_checking_surface_index))
                        cmd.extend(torso_pose_grid.cell_3D_list[i][j][k].right_hand_checking_surface_index)

            # import IPython; IPython.embed()
            for key,window in torso_pose_grid.left_foot_neighbor_window.iteritems():
                cmd.append(len(window))
                for cell in window:
                    cmd.extend(cell)

            for key,window in torso_pose_grid.right_foot_neighbor_window.iteritems():
                cmd.append(len(window))
                for cell in window:
                    cmd.extend(cell)

            for key,window in torso_pose_grid.chest_neighbor_window.iteritems():
                cmd.append(len(window))
                for cell in window:
                    cmd.extend(cell)

        if hand_transition_model is not None:
            cmd.append('hand_transition_model')
            cmd.append(len(hand_transition_model))

            for hand_transition in hand_transition_model:
                cmd.extend(hand_transition)

        if parallelization is not None:
            cmd.append('parallelization')

            if parallelization:
                cmd.append(1)
            else:
                cmd.append(0)

        cmd_str = " ".join(str(item) for item in cmd)

        after_constructing_command = time.time()

        result_str = self.module.SendCommand(cmd_str)

        after_calculation = time.time()

        result = [float(x) for x in result_str.split()]

        footstep_transition_traversability_legs_only = {}
        footstep_transition_traversability = {}
        hand_transition_traversability = {}

        counter = 0

        footstep_transition_num = result[counter]
        counter += 1
        for i in range(int(footstep_transition_num)):
            footstep_transition_traversability_legs_only[tuple(int(x) for x in result[counter:counter+5])] = result[counter+5:counter+8]
            counter += 8

        footstep_transition_num = result[counter]
        counter += 1
        for i in range(int(footstep_transition_num)):
            footstep_transition_traversability[tuple(int(x) for x in result[counter:counter+5])] = result[counter+5:counter+8]
            counter += 8

        hand_transition_num = result[counter]
        counter += 1
        for i in range(int(hand_transition_num)):
            hand_transition_traversability[tuple(int(x) for x in result[counter:counter+3])] = result[counter+3:counter+15]
            counter += 15


        after_parsing_output = time.time()

        print('Constructing Command Time: %d miliseconds.'%((after_constructing_command-start)*1000))
        print('Calculation Time: %d miliseconds.'%((after_calculation-after_constructing_command)*1000))
        print('Parsing Output Time: %d miliseconds.'%((after_parsing_output-after_calculation)*1000))

        return (footstep_transition_traversability_legs_only,footstep_transition_traversability,hand_transition_traversability)


    def SendStartConstructingContactRegions(self, structures=None, printing=False, structures_id=None):

        start = time.time()

        cmd = ['StartConstructingContactRegions']

        if printing:
            cmd.append('printing')

        if structures is not None:
            cmd.append('structures')
            cmd.append(len(structures))

            for struct in structures:
                cmd.append(struct.geometry)
                cmd.append(struct.kinbody.GetName())
                cmd.append(struct.id)

                # plane parameters
                cmd.extend((struct.nx,struct.ny,struct.nz,struct.c))

                # vertices
                cmd.append(len(struct.vertices))

                for vertex in struct.vertices:
                    cmd.extend(vertex)

                # boundaries
                cmd.append(len(struct.boundaries))

                for edge in struct.boundaries:
                    cmd.extend(edge)

                # trimesh types
                cmd.append(struct.type)


        if structures_id is not None:
            cmd.append('structures_id')
            cmd.append(len(structures_id))
            for s_id in structures_id:
                cmd.append(s_id)


        cmd_str = " ".join(str(item) for item in cmd)

        after_constructing_command = time.time()

        result_str = self.module.SendCommand(cmd_str)

        after_calculation = time.time()

        result = [float(x) for x in result_str.split()]

        counter = 0

        contact_points_num = int(result[counter])
        contact_points_values = [[0]*6 for i in range(contact_points_num)]
        counter += 1
        for i in range(contact_points_num):
            contact_points_values[i] = result[counter:counter+6]
            counter += 6

        contact_regions_num = int(result[counter])
        contact_regions_values = [[0]*7 for i in range(contact_regions_num)]
        counter += 1
        for i in range(contact_regions_num):
            contact_regions_values[i] = result[counter:counter+7]
            counter += 7

        after_parsing_output = time.time()

        print('Constructing Command Time: %d miliseconds.'%((after_constructing_command-start)*1000))
        print('Contact Region and Point Calculation Time: %d miliseconds.'%((after_calculation-after_constructing_command)*1000))
        print('Parsing Output Time: %d miliseconds.'%((after_parsing_output-after_calculation)*1000))

        return (contact_points_values,contact_regions_values)
