
import numpy as np
import openravepy as rave
import random
import math
import pickle
import IPython
import sys

from config_parameter import *
from drawing_functions import *
from transformation_conversion import *
from structures import *

class environment_handler:
    def __init__(self,mode='surface_world',env=None,structures=None):
        if env is None:
            self.env = rave.Environment()  # create openrave environment
            self.env.SetViewer('qtcoin')  # attach viewer (optional)
            self.env.SetCollisionChecker(rave.RaveCreateCollisionChecker(self.env,'ode'))
        else:
            self.env = env

        self.surface_list = None
        self.surface_mesh_list = None
        self.structures = structures

        self.mode = mode

        self.intersecting_points = None

        self.start_dist_to_boundary = 0
        self.goal_dist_to_boundary = 0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.init_z = -9999.0
        self.goal_z = -9999.0


    def extend_trivial_corridor(self,structures,corridor_transform,corridor_dimension):
        ground_l = corridor_dimension[0]
        ground_w = corridor_dimension[1]

        surface_boundaries = [(0,1),(1,2),(2,3),(3,0)]
        surface_trimesh_indices = np.array([[0,1,2],[0,2,3]])

        init_surface_vertices = [(0,ground_w/2,0),(0,-ground_w/2,0),(ground_l,-ground_w/2,0),(ground_l,ground_w/2,0)]
        surface_vertices = [None] * 4

        surface_transform_matrix = corridor_transform

        for i in range(4):
            surface_vertex = np.dot(surface_transform_matrix,np.array([[init_surface_vertices[i][0]],[init_surface_vertices[i][1]],[0],[1]],dtype=float))

            surface_vertices[i] = (surface_vertex[0,0],surface_vertex[1,0],surface_vertex[2,0])

        corridor_transform_matrix = corridor_transform

        surface_trimesh_vertices = np.zeros((4,3),dtype=float)
        for i in range(len(surface_trimesh_vertices)):
            for j in range(3):
                surface_trimesh_vertices[i,j] = surface_vertices[i][j]

        surface_trimesh = rave.RaveCreateKinBody(self.env,'')
        surface_trimesh.SetName('random_trimesh_' + str(len(structures)))
        surface_trimesh.InitFromTrimesh(rave.TriMesh(surface_trimesh_vertices,surface_trimesh_indices),False)

        if corridor_transform_matrix[2,2] >= 0:
            surface_plane_parameter = [corridor_transform_matrix[0,2],corridor_transform_matrix[1,2],corridor_transform_matrix[2,2],-(np.dot(corridor_transform_matrix[0:3,2],corridor_transform_matrix[0:3,3]))]
        else:
            surface_plane_parameter = [-corridor_transform_matrix[0,2],-corridor_transform_matrix[1,2],-corridor_transform_matrix[2,2],np.dot(corridor_transform_matrix[0:3,2],corridor_transform_matrix[0:3,3])]

        random_surface = trimesh_surface(len(structures),surface_plane_parameter,
                                         surface_vertices,
                                         surface_boundaries,
                                         surface_trimesh,
                                         surface_trimesh_vertices,
                                         surface_trimesh_indices)

        random_surface.type = 'ground'

        self.env.AddKinBody(random_surface.kinbody)

        structures.append(random_surface)
        # DrawOrientation(self.env,random_surface.transform)
        DrawSurface(self.env,random_surface)

        wall_r_1 = ground_w/2
        wall_theta_1 = -math.pi/2
        wall_r_2 = ground_w/2
        wall_theta_2 = math.pi/2

        wall_position = [(wall_r_1,wall_theta_1),(wall_r_2,wall_theta_2)]


        for wp in wall_position:
            wall_r = wp[0]
            wall_theta = wp[1]
            wall_theta_deg = wall_theta*rad_to_deg

            wall_center = [wall_r*math.cos(wall_theta)+ground_l/2,wall_r*math.sin(wall_theta)]
            wall_dir = [-math.sin(wall_theta),math.cos(wall_theta)]


            surface_transform = [wall_center[0],wall_center[1],1.45,-90,-90-wall_theta_deg,0]
            init_surface_vertices = [(-ground_l/2,ground_w/2,0),(-ground_l/2,-ground_w/2,0),(ground_l/2,-ground_w/2,0),(ground_l/2,ground_w/2,0)]
            surface_vertices = [None] * 4

            surface_transform_matrix = np.dot(corridor_transform,xyzrpy_to_SE3(surface_transform))

            for i in range(4):
                surface_vertex = np.dot(surface_transform_matrix,np.array([[init_surface_vertices[i][0]],[init_surface_vertices[i][1]],[0],[1]],dtype=float))

                surface_vertices[i] = (surface_vertex[0,0],surface_vertex[1,0],surface_vertex[2,0])

            surface_trimesh_vertices = np.zeros((4,3),dtype=float)
            for i in range(len(surface_trimesh_vertices)):
                for j in range(3):
                    surface_trimesh_vertices[i,j] = surface_vertices[i][j]

            surface_trimesh = rave.RaveCreateKinBody(self.env,'')
            surface_trimesh.SetName('random_trimesh_' + str(len(structures)))
            surface_trimesh.InitFromTrimesh(rave.TriMesh(surface_trimesh_vertices,surface_trimesh_indices),False)

            surface_plane_parameter = [surface_transform_matrix[0,2],surface_transform_matrix[1,2],surface_transform_matrix[2,2],-(np.dot(surface_transform_matrix[0:3,2],surface_transform_matrix[0:3,3]))]

            random_surface = trimesh_surface(len(structures),surface_plane_parameter,
                                             surface_vertices,
                                             surface_boundaries,
                                             surface_trimesh,
                                             surface_trimesh_vertices,
                                             surface_trimesh_indices)

            random_surface.type = 'others'

            self.env.AddKinBody(random_surface.kinbody)

            structures.append(random_surface)
            # DrawOrientation(self.env,random_surface.transform)
            DrawSurface(self.env,random_surface)

    def pave_tilted_surfaces(self, structures, origin_pose, surface_spacing, max_tilt_angle, area_size):
        origin_x = origin_pose[0]
        origin_y = origin_pose[1]
        origin_z = origin_pose[2]

        pab = max_tilt_angle

        surface_r_bound = (0.35,0.45)
        surface_theta_bound = [(22.5,67.5),(112.5,157.5),(202.5,247.5),(292.5,337.5)]
        surface_boundaries = [(0,1),(1,2),(2,3),(3,0)]
        surface_trimesh_indices = np.array([[0,1,2],[0,2,3]])

        patch_dim_x = int(round(area_size[0] / surface_spacing))
        patch_dim_y = int(round(area_size[1] / surface_spacing))

        for ix in range(patch_dim_x):
            for iy in range(patch_dim_y):

                x = origin_x + (ix+0.5) * surface_spacing
                y = origin_y + (iy+0.5) * surface_spacing
                z = origin_z

                surface_transform = [None] * 6
                surface_vertices = [None] * 4

                surface_transform_bound = [(x-0.0,x+0.0), (y-0.0,y+0.0), (z-0.05,z+0.05), (-pab,pab), (-pab,pab), (-180,180)]

                surface_transform = [(surface_transform_bound[i][1] - surface_transform_bound[i][0]) * random.random() + surface_transform_bound[i][0] for i in range(6)]

                surface_transform_matrix = xyzrpy_to_SE3(surface_transform)

                for i in range(4):
                    r = (surface_r_bound[1] - surface_r_bound[0]) * random.random() + surface_r_bound[0]
                    theta = ((surface_theta_bound[i][1] - surface_theta_bound[i][0]) * random.random() + surface_theta_bound[i][0]) * deg_to_rad

                    surface_vertex = np.dot(surface_transform_matrix,np.array([[r*math.cos(theta)],[r*math.sin(theta)],[0],[1]],dtype=float))

                    surface_vertices[i] = (surface_vertex[0,0],surface_vertex[1,0],surface_vertex[2,0])

                surface_trimesh_vertices = np.array(surface_vertices)

                surface_trimesh = rave.RaveCreateKinBody(self.env,'')
                surface_trimesh.SetName('random_trimesh_' + str(len(structures)))
                surface_trimesh.InitFromTrimesh(rave.TriMesh(surface_trimesh_vertices,surface_trimesh_indices),False)

                if surface_transform_matrix[2,2] >= 0:
                    surface_plane_parameter = [surface_transform_matrix[0,2],surface_transform_matrix[1,2],surface_transform_matrix[2,2],-(np.dot(surface_transform_matrix[0:3,2],surface_transform_matrix[0:3,3]))]
                else:
                    surface_plane_parameter = [-surface_transform_matrix[0,2],-surface_transform_matrix[1,2],-surface_transform_matrix[2,2],np.dot(surface_transform_matrix[0:3,2],surface_transform_matrix[0:3,3])]

                random_surface = trimesh_surface(len(structures),surface_plane_parameter,
                                                 surface_vertices,
                                                 surface_boundaries,
                                                 surface_trimesh,
                                                 surface_trimesh_vertices,
                                                 surface_trimesh_indices)

                random_surface.type = 'ground'

                self.env.AddKinBody(random_surface.kinbody)

                structures.append(random_surface)
                # DrawOrientation(self.env,random_surface.transform)
                DrawSurface(self.env,random_surface)

    def pave_flat_surfaces(self, structures, origin_pose, area_size):
        surface_lx = area_size[0]
        surface_ly = area_size[1]

        surface_boundaries = [(0,1),(1,2),(2,3),(3,0)]
        surface_trimesh_indices = np.array([[0,1,2],[0,2,3]])

        init_surface_vertices = [(0,0,0),(surface_lx,0,0),(surface_lx,surface_ly,0),(0,surface_ly,0)]
        surface_vertices = [None] * 4

        surface_transform_matrix = xyzrpy_to_SE3(origin_pose)

        for i in range(4):
            surface_vertex = np.dot(surface_transform_matrix,np.array([[init_surface_vertices[i][0]],[init_surface_vertices[i][1]],[0],[1]],dtype=float))
            surface_vertices[i] = (surface_vertex[0,0],surface_vertex[1,0],surface_vertex[2,0])


        surface_trimesh_vertices = np.array(surface_vertices)

        surface_trimesh = rave.RaveCreateKinBody(self.env,'')
        surface_trimesh.SetName('flat_surface_' + str(len(structures)))
        surface_trimesh.InitFromTrimesh(rave.TriMesh(surface_trimesh_vertices,surface_trimesh_indices),False)

        if surface_transform_matrix[2,2] >= 0:
            surface_plane_parameter = [surface_transform_matrix[0,2],surface_transform_matrix[1,2],surface_transform_matrix[2,2],-(np.dot(surface_transform_matrix[0:3,2],surface_transform_matrix[0:3,3]))]
        else:
            surface_plane_parameter = [-surface_transform_matrix[0,2],-surface_transform_matrix[1,2],-surface_transform_matrix[2,2],np.dot(surface_transform_matrix[0:3,2],surface_transform_matrix[0:3,3])]

        flat_surface = trimesh_surface(len(structures),surface_plane_parameter,
                                       surface_vertices,
                                       surface_boundaries,
                                       surface_trimesh,
                                       surface_trimesh_vertices,
                                       surface_trimesh_indices)

        flat_surface.type = 'ground'

        self.env.AddKinBody(flat_surface.kinbody)

        structures.append(flat_surface)
        # DrawOrientation(self.env,flat_surface.transform)
        DrawSurface(self.env,flat_surface)

    def extend_mixed_corridor(self, structures, corridor_pose, patch_size, corridor_length, max_tilt_angle=20):

        origin_x = corridor_pose[0]
        origin_y = corridor_pose[1]
        origin_z = corridor_pose[2]
        yaw = corridor_pose[5]

        half_ground_dist = patch_size/2.0
        half_wall_dist = half_ground_dist

        wall_length = corridor_length

        corridor_transform_matrix = xyzrpy_to_SE3(corridor_pose)

        nx = corridor_transform_matrix[0:3,0]
        ny = corridor_transform_matrix[0:3,1]

        self.construct_mixed_wall(structures, [origin_x-half_wall_dist*ny[0], origin_y-half_wall_dist*ny[1],origin_z,0,0,yaw],
                                  patch_size, 20, wall_length, [])

        self.construct_mixed_wall(structures, [origin_x+wall_length*nx[0]+half_wall_dist*ny[0], origin_y+wall_length*nx[1]+half_wall_dist*ny[1],origin_z,0,0,yaw+180],
                                  patch_size, 20, wall_length, [])

        patch_num = int(round(corridor_length/patch_size))
        ground_flat = [(random.random() > tilted_surface_probability) for j in range(patch_num)]
        # ground_flat = [(random.random() > 1.0) for j in range(patch_num)]
        ground_counted = [False for j in range(patch_num)]

        ground_origin_pose = [origin_x-half_ground_dist*ny[0], origin_y-half_ground_dist*ny[1],origin_z,0,0,yaw]
        ground_origin_pose_matrix = xyzrpy_to_SE3(ground_origin_pose)

        for j in range(patch_num):

            if ground_flat[j] is True and ground_counted[j] is False:
                ground_counted[j] = True
                flat_ground_length = patch_size
                flat_ground_origin_pose = copy.deepcopy(ground_origin_pose)

                for nj in range(j+1,patch_num):
                    if ground_flat[nj] is True and ground_counted[nj] is False:
                        ground_counted[nj] = True
                        flat_ground_length += patch_size
                    else:
                        break

                flat_ground_origin_pose[0] = flat_ground_origin_pose[0] + j * patch_size * ground_origin_pose_matrix[0,0]
                flat_ground_origin_pose[1] = flat_ground_origin_pose[1] + j * patch_size * ground_origin_pose_matrix[1,0]

                self.pave_flat_surfaces(structures, flat_ground_origin_pose, (flat_ground_length,patch_size))

            elif ground_flat[j] is False:
                tilted_ground_origin_pose = copy.deepcopy(ground_origin_pose)

                tilted_ground_length = patch_size
                tilted_ground_origin_pose[0] = tilted_ground_origin_pose[0] + j * patch_size * ground_origin_pose_matrix[0,0]
                tilted_ground_origin_pose[1] = tilted_ground_origin_pose[1] + j * patch_size * ground_origin_pose_matrix[1,0]

                self.pave_tilted_surfaces(structures, tilted_ground_origin_pose, 0.5, max_tilt_angle, (tilted_ground_length,tilted_ground_length))

    def extend_mixed_stair(self,structures,stair_pose,stair_length,stair_width=1.5,stair_step_depth=1.0,stair_step_height=0.1,max_tilt_angle=20):

        origin_x = stair_pose[0]
        origin_y = stair_pose[1]
        origin_z = stair_pose[2]
        yaw = stair_pose[5]

        half_ground_dist = stair_width/2.0
        half_wall_dist = half_ground_dist

        wall_length = stair_length

        stair_transform_matrix = xyzrpy_to_SE3(stair_pose)

        nx = stair_transform_matrix[0:3,0]
        ny = stair_transform_matrix[0:3,1]

        wall_patch_size = 1.5

        self.construct_mixed_wall(structures, [origin_x-half_wall_dist*ny[0], origin_y-half_wall_dist*ny[1],origin_z,0,0,yaw],
                                  wall_patch_size, 20, wall_length, [], slope=-(stair_step_height/stair_step_depth))

        self.construct_mixed_wall(structures, [origin_x+wall_length*nx[0]+half_wall_dist*ny[0], origin_y+wall_length*nx[1]+half_wall_dist*ny[1],origin_z+wall_length*(stair_step_height/stair_step_depth),0,0,yaw+180],
                                  wall_patch_size, 20, wall_length, [], slope=(stair_step_height/stair_step_depth))

        patch_num = int(round(stair_length/stair_step_depth))
        ground_flat = [(random.random() > tilted_surface_probability) for j in range(patch_num)]
        # ground_flat = [(random.random() > 1.0) for j in range(patch_num)]
        ground_counted = [False for j in range(patch_num)]

        ground_origin_pose = [origin_x-half_ground_dist*ny[0], origin_y-half_ground_dist*ny[1],origin_z,0,0,yaw]
        ground_origin_pose_matrix = xyzrpy_to_SE3(ground_origin_pose)

        for j in range(patch_num):

            if ground_flat[j] is True and ground_counted[j] is False:
                ground_counted[j] = True
                flat_ground_length = stair_step_depth
                flat_ground_origin_pose = copy.deepcopy(ground_origin_pose)

                flat_ground_origin_pose[0] = flat_ground_origin_pose[0] + j * flat_ground_length * ground_origin_pose_matrix[0,0]
                flat_ground_origin_pose[1] = flat_ground_origin_pose[1] + j * stair_width * ground_origin_pose_matrix[1,0]
                flat_ground_origin_pose[2] = flat_ground_origin_pose[2] + j * stair_step_height + origin_z

                self.pave_flat_surfaces(structures, flat_ground_origin_pose, (flat_ground_length,stair_width))

            elif ground_flat[j] is False:
                tilted_ground_origin_pose = copy.deepcopy(ground_origin_pose)

                tilted_ground_length = stair_step_depth
                tilted_ground_origin_pose[0] = tilted_ground_origin_pose[0] + j * tilted_ground_length * ground_origin_pose_matrix[0,0]
                tilted_ground_origin_pose[1] = tilted_ground_origin_pose[1] + j * stair_width * ground_origin_pose_matrix[1,0]
                tilted_ground_origin_pose[2] = tilted_ground_origin_pose[2] + j * stair_step_height + origin_z

                self.pave_tilted_surfaces(structures, tilted_ground_origin_pose, 0.5, max_tilt_angle, (tilted_ground_length,stair_width))

    def construct_flat_wall(self, structures, origin_pose, wall_length, slope=0):

        origin_pose[2] += 1.45 # z

        wall_boundaries = [(0,1),(1,2),(2,3),(3,0)]
        wall_transform_matrix = np.dot(xyzrpy_to_SE3(origin_pose), xyzrpy_to_SE3([wall_length/2.0,0,0,-90,0,0]))
        init_wall_vertices = [(-wall_length/2.0,0.5,0),
                              (-wall_length/2.0,-0.5,0),
                              (wall_length/2.0,-0.5+wall_length*slope,0),
                              (wall_length/2.0,0.5+wall_length*slope,0)]
        wall_vertices = [None] * 4

        for i in range(4):
            wall_vertex = np.dot(wall_transform_matrix,np.array([[init_wall_vertices[i][0]],[init_wall_vertices[i][1]],[0],[1]],dtype=float))

            wall_vertices[i] = (wall_vertex[0,0],wall_vertex[1,0],wall_vertex[2,0])

        wall_trimesh_vertices = np.array(wall_vertices)
        wall_trimesh_indices = np.array([[0,1,2],[0,2,3]])

        wall_trimesh = rave.RaveCreateKinBody(self.env,'')
        wall_trimesh.SetName('random_trimesh_' + str(len(structures)))
        wall_trimesh.InitFromTrimesh(rave.TriMesh(wall_trimesh_vertices,wall_trimesh_indices),False)

        wall_plane_parameter = [wall_transform_matrix[0,2],wall_transform_matrix[1,2],wall_transform_matrix[2,2],-(np.dot(wall_transform_matrix[0:3,2],wall_transform_matrix[0:3,3]))]

        wall_surface = trimesh_surface(len(structures),wall_plane_parameter,wall_vertices,wall_boundaries,wall_trimesh,wall_trimesh_vertices,wall_trimesh_indices)
        wall_surface.type = 'others'

        self.env.AddKinBody(wall_surface.kinbody)

        structures.append(wall_surface)
        # DrawOrientation(self.env,wall_surface.transform)
        DrawSurface(self.env,wall_surface,0.5)

        # IPython.embed()

    def construct_tilted_wall(self, structures, origin_pose, wall_spacing, max_tilted_angle, wall_length, slope=0):

        surface_r_bound = (0.35,0.45)
        surface_theta_bound = [(22.5,67.5),(112.5,157.5),(202.5,247.5),(292.5,337.5)]
        surface_boundaries = [(0,1),(1,2),(2,3),(3,0)]
        surface_trimesh_indices = np.array([[0,1,2],[0,2,3]])

        pab = max_tilted_angle

        surface_num = int(round(wall_length/wall_spacing))

        for ix in range(surface_num):
            x = (ix+0.5) * wall_spacing

            surface_transform_bound = [(x,x), (0.0,0.0), (1.35-x*slope,1.55-x*slope), (-90-pab,-90+pab), (-pab,pab), (-180,180)]

            surface_transform = [None] * 6
            surface_vertices = [None] * 4

            for i in range(6):
                surface_transform[i] = (surface_transform_bound[i][1] - surface_transform_bound[i][0]) * random.random() + surface_transform_bound[i][0]

            surface_transform_matrix = np.dot(xyzrpy_to_SE3(origin_pose), xyzrpy_to_SE3(surface_transform))

            for i in range(4):
                r = (surface_r_bound[1] - surface_r_bound[0]) * random.random() + surface_r_bound[0]
                theta = ((surface_theta_bound[i][1] - surface_theta_bound[i][0]) * random.random() + surface_theta_bound[i][0]) * deg_to_rad

                surface_vertex = np.dot(surface_transform_matrix,np.array([[r*math.cos(theta)],[r*math.sin(theta)],[0],[1]],dtype=float))

                surface_vertices[i] = (surface_vertex[0,0],surface_vertex[1,0],surface_vertex[2,0])

            surface_trimesh_vertices = np.array(surface_vertices)

            surface_trimesh = rave.RaveCreateKinBody(self.env,'')
            surface_trimesh.SetName('tilted_wall_surface_' + str(len(structures)))
            surface_trimesh.InitFromTrimesh(rave.TriMesh(surface_trimesh_vertices,surface_trimesh_indices),False)

            surface_type = 'others'

            surface_plane_parameter = [surface_transform_matrix[0,2],surface_transform_matrix[1,2],surface_transform_matrix[2,2],-(np.dot(surface_transform_matrix[0:3,2],surface_transform_matrix[0:3,3]))]

            wall_surface = trimesh_surface(len(structures),surface_plane_parameter,surface_vertices,surface_boundaries,surface_trimesh,surface_trimesh_vertices,surface_trimesh_indices)
            wall_surface.type = surface_type

            self.env.AddKinBody(wall_surface.kinbody)

            structures.append(wall_surface)
            # DrawOrientation(self.env,wall_surface.transform)
            DrawSurface(self.env,wall_surface,0.5)

            # raw_input('wall surface, x = %5.3f, y = %5.3f.'%(x,surface_transform[1]))

    def construct_mixed_wall(self, structures, origin_pose, patch_size, max_tilted_angle, wall_length, doors=None, slope=0):

        patch_num = int(round(wall_length/patch_size))
        wall_flat = [(random.random() > tilted_surface_probability) for j in range(patch_num)]
        # wall_flat = [(random.random() > 1.0) for j in range(patch_num)]
        wall_counted = [False for j in range(patch_num)]

        origin_pose_matrix = xyzrpy_to_SE3(origin_pose)

        for door in doors:
            wall_flat[door] = None

        for j in range(patch_num):

            if wall_flat[j] is True and wall_counted[j] is False:
                wall_counted[j] = True
                flat_wall_length = patch_size
                flat_wall_origin_pose = copy.deepcopy(origin_pose)

                for nj in range(j+1,patch_num):
                    if wall_flat[nj] is True and wall_counted[nj] is False:
                        wall_counted[nj] = True
                        flat_wall_length += patch_size
                    else:
                        break

                flat_wall_origin_pose[0] = flat_wall_origin_pose[0] + j * patch_size * origin_pose_matrix[0,0]
                flat_wall_origin_pose[1] = flat_wall_origin_pose[1] + j * patch_size * origin_pose_matrix[1,0]
                flat_wall_origin_pose[2] = flat_wall_origin_pose[2] - j * patch_size * math.hypot(origin_pose_matrix[0,0],origin_pose_matrix[1,0]) * slope

                self.construct_flat_wall(structures, flat_wall_origin_pose, flat_wall_length, slope=slope)

            elif wall_flat[j] is False:
                tilted_wall_origin_pose = copy.deepcopy(origin_pose)

                tilted_wall_length = patch_size
                tilted_wall_origin_pose[0] = tilted_wall_origin_pose[0] + j * patch_size * origin_pose_matrix[0,0]
                tilted_wall_origin_pose[1] = tilted_wall_origin_pose[1] + j * patch_size * origin_pose_matrix[1,0]
                tilted_wall_origin_pose[2] = tilted_wall_origin_pose[2] - j * patch_size * math.hypot(origin_pose_matrix[0,0],origin_pose_matrix[1,0]) * slope

                self.construct_tilted_wall(structures, tilted_wall_origin_pose, 0.5, max_tilted_angle, tilted_wall_length, slope=slope)


    def construct_room(self, structures, origin, patch_size, room_size, doors, force_flat_patch=[]):
        origin_x = origin[0]
        origin_y = origin[1]

        if len(origin) > 2:
            origin_z = origin[2]
        else:
            origin_z = 0

        room_dim_x = int(round(room_size[0] / patch_size))
        room_dim_y = int(round(room_size[1] / patch_size))

        # construct the ground

        # decide each patch is flat or not
        patch_flat = [[(random.random() > tilted_surface_probability) for j in range(room_dim_y)] for i in range(room_dim_x)]
        patch_counted = [[False for j in range(room_dim_y)] for i in range(room_dim_x)]

        for patch in force_flat_patch:
            patch_flat[patch[0]][patch[1]] = True

        for i in range(room_dim_x):
            for j in range(room_dim_y):
                if patch_flat[i][j] is True and patch_counted[i][j] is False:
                    patch_counted[i][j] = True
                    connected_flat_patches = [(i,j)]
                    open_connected_flat_patches = [(i,j)]

                    while open_connected_flat_patches:
                        (ci,cj) = open_connected_flat_patches.pop();
                        for ni in range(ci-1,ci+2):
                            for nj in range(cj-1,cj+2):
                                if ni >= 0 and ni < room_dim_x and nj >= 0 and nj < room_dim_y and (ni == ci or nj == cj):
                                    if patch_flat[ni][nj] is True and patch_counted[ni][nj] is False:
                                        patch_counted[ni][nj] = True
                                        connected_flat_patches.append((ni,nj))
                                        open_connected_flat_patches.append((ni,nj))

                    vertices_counts = {}
                    trimesh_vertices = set()
                    trimesh_indices = []

                    boundary_vertices = []
                    boundary_indices = []

                    for patch in connected_flat_patches:
                        pi = patch[0]
                        pj = patch[1]
                        patch_vertices = [(origin_x+pi*patch_size, origin_y+pj*patch_size,origin_z),
                                          (origin_x+(pi+1)*patch_size, origin_y+pj*patch_size,origin_z),
                                          (origin_x+(pi+1)*patch_size, origin_y+(pj+1)*patch_size,origin_z),
                                          (origin_x+pi*patch_size, origin_y+(pj+1)*patch_size,origin_z)]

                        for vertex in patch_vertices:
                            trimesh_vertices.add(vertex)
                            if vertices_counts.get(vertex) is None:
                                vertices_counts[vertex] = 1
                            else:
                                vertices_counts[vertex] = vertices_counts[vertex] + 1

                    trimesh_vertices = list(trimesh_vertices)

                    trimesh_vertices_indices_dict = {}
                    for k,vertex in enumerate(trimesh_vertices):
                        trimesh_vertices_indices_dict[vertex] = k

                        if vertices_counts[vertex] < 4:
                            boundary_vertices.append(vertex)


                    boundary_vertices_indices_dict = {}
                    for k,vertex in enumerate(boundary_vertices):
                        boundary_vertices_indices_dict[vertex] = k

                    edge_count = {}

                    # next_index_dict = {}
                    for patch in connected_flat_patches:
                        pi = patch[0]
                        pj = patch[1]
                        patch_vertices = [(origin_x+pi*patch_size, origin_y+pj*patch_size,origin_z),
                                          (origin_x+(pi+1)*patch_size, origin_y+pj*patch_size,origin_z),
                                          (origin_x+(pi+1)*patch_size, origin_y+(pj+1)*patch_size,origin_z),
                                          (origin_x+pi*patch_size, origin_y+(pj+1)*patch_size,origin_z)]

                        patch_trimesh_indices = [trimesh_vertices_indices_dict[vertex] for vertex in patch_vertices]
                        trimesh_indices.append((patch_trimesh_indices[0],patch_trimesh_indices[1],patch_trimesh_indices[2]))
                        trimesh_indices.append((patch_trimesh_indices[0],patch_trimesh_indices[2],patch_trimesh_indices[3]))

                        for k in range(len(patch_vertices)):
                            vertex1 = patch_vertices[k%4]
                            vertex2 = patch_vertices[(k+1)%4]

                            if vertex1 in boundary_vertices_indices_dict and vertex2 in boundary_vertices_indices_dict:
                                boundary_index1 = boundary_vertices_indices_dict[vertex1]
                                boundary_index2 = boundary_vertices_indices_dict[vertex2]

                                if (boundary_index1,boundary_index2) in edge_count:
                                    edge_count[(boundary_index1,boundary_index2)] += 1
                                elif (boundary_index2,boundary_index1) in edge_count:
                                    edge_count[(boundary_index2,boundary_index1)] += 1
                                else:
                                    edge_count[(boundary_index1,boundary_index2)] = 1

                    for edge_vetex_indiex_pair, count in edge_count.iteritems():
                        if count == 1:
                            boundary_indices.append(edge_vetex_indiex_pair)


                    # next_index = next_index_dict[0]
                    # boundary_indices.append((0,next_index))
                    # while next_index != 0:
                    #     current_index = next_index
                    #     next_index = next_index_dict[current_index]
                    #     boundary_indices.append((current_index,next_index))


                    openrave_trimesh_vertices = np.array(trimesh_vertices)
                    openrave_trimesh_indices = np.array(trimesh_indices)

                    flat_ground_trimesh = rave.RaveCreateKinBody(self.env,'')
                    flat_ground_trimesh.SetName('flat_ground_' + str(len(structures)))
                    flat_ground_trimesh.InitFromTrimesh(rave.TriMesh(openrave_trimesh_vertices,openrave_trimesh_indices),False)

                    start_neighbor_ground = trimesh_surface(len(structures),[0,0,1.0,-origin_z],boundary_vertices,boundary_indices,flat_ground_trimesh,openrave_trimesh_vertices,openrave_trimesh_indices)

                    self.env.AddKinBody(start_neighbor_ground.kinbody)
                    structures.append(start_neighbor_ground)
                    DrawSurface(self.env,start_neighbor_ground)

                    print(connected_flat_patches)

                elif patch_flat[i][j] is False: # tilted surfaces
                    self.pave_tilted_surfaces(structures, [origin_x+i*patch_size,origin_y+j*patch_size,origin_z,0,0,0], 0.5, 20, (1.5,1.5))

        DrawOrientation(self.env,np.eye(4))

        # construct the wall
        wall_patch_num = [room_dim_x, room_dim_y, room_dim_x, room_dim_y]

        for i in range(4):

            wall_length = patch_size * wall_patch_num[i]

            if i == 0:
                x = origin_x; y = origin_y; z = origin_z; yaw = 0
            elif i == 1:
                x = origin_x + room_dim_x * patch_size; y = origin_y; z = origin_z; yaw = 90
            elif i == 2:
                x = origin_x + room_dim_x * patch_size; y = origin_y + room_dim_y * patch_size; z = origin_z; yaw = 180
            elif i == 3:
                x = origin_x; y = origin_y + room_dim_y * patch_size; z = origin_z; yaw = 270

            self.construct_mixed_wall(structures, [x,y,z,0,0,yaw], patch_size, 20, wall_length, doors[i])


    def update_environment(self,or_robot,file_path=None,surface_source='flat_ground_environment'):

        if self.structures is not None:
            for struct in self.structures:
                if struct.kinbody is not None:
                    self.env.Remove(struct.kinbody)


        global draw_handles

        for i in range(len(draw_handles)):
            if draw_handles[i] is not None:
                draw_handles[i].SetShow(False)
                draw_handles[i].Close()
                draw_handles[i] = None

        if self.mode == 'surface_world':

            structures = []

            if surface_source == 'load_from_data':

                if file_path is None:
                    print('No file path provided.')
                    self.structures = []
                else:
                    out_file = open(file_path,'r')
                    (structures,self.goal_x,self.goal_y,_) = pickle.load(out_file)
                    out_file.close()

                    # reconstruct the trimesh surfaces
                    for i in range(len(structures)):
                        surface_trimesh = rave.RaveCreateKinBody(self.env,'')
                        surface_trimesh.SetName('surface_' + str(i))

                        surface_trimesh.InitFromTrimesh(rave.TriMesh(structures[i].trimesh_vertices,structures[i].trimesh_indices),False)

                        structures[i].kinbody = surface_trimesh

                        self.env.AddKinBody(structures[i].kinbody)

                        if structures[i].type == 'others':
                            DrawSurface(self.env,structures[i],0.5)
                        else:
                            DrawSurface(self.env,structures[i])

                    max_x = sys.float_info.min
                    min_x = sys.float_info.max
                    max_y = sys.float_info.min
                    min_y = sys.float_info.max

                    for struct in structures:
                        for vertex in struct.vertices:
                            if vertex[0] > max_x:
                                max_x = vertex[0]

                            if vertex[0] < min_x:
                                min_x = vertex[0]

                            if vertex[1] > max_y:
                                max_y = vertex[1]

                            if vertex[1] < min_y:
                                min_y = vertex[1]

                    if self.env.GetViewer() is not None:
                        self.env.GetViewer().SetCamera([
                            [ 0, -1,  0, (max_x+min_x)/2.0],
                            [-1,  0,  0, (max_y+min_y)/2.0],
                            [ 0,  0, -1, 19.0],
                            [0., 0., 0., 1.]])

            elif surface_source == 'extended_trivial_corridor':
                self.goal_x = 6.7
                self.goal_y = 0.0

                self.extend_trivial_corridor(structures,corridor_transform=xyzrpy_to_SE3([-0.3,0,0,0,0,0]),corridor_dimension=(7.5,1.5))

            elif surface_source == 'two_corridor_environment':
                # set a patch size as 1.5 x 1.5 m

                env_anchor_x = random.random() * (-1.0-(-3.5)) - 3.5
                env_anchor_y = random.random() * (-0.75-(-5)) - 5

                patch_size = 1.5

                while(abs(-env_anchor_x-1.5) <= or_robot.foot_h/2.0+0.05 or
                      abs(-env_anchor_x-3.0) <= or_robot.foot_h/2.0+0.05 or
                      abs(-env_anchor_y-1.5) <= or_robot.foot_w+0.07 or
                      abs(-env_anchor_y-3.0) <= or_robot.foot_w+0.07 or
                      abs(-env_anchor_y-4.5) <= or_robot.foot_w+0.07):
                    env_anchor_x = random.random() * (-1.0-(-3.5)) - 3.5
                    env_anchor_y = random.random() * (-0.75-(-5)) - 5

                self.goal_x = env_anchor_x + random.random() * (12.5 - 9.0) + 9.0
                self.goal_y = env_anchor_y + random.random() * (5.25 - 0.75) + 0.75


                self.goal_x = 3.0
                self.goal_y = 0

                env_anchor_x = -1.2
                env_anchor_y = -0.75

                first_room_x_dimension = 4.5
                first_room_y_dimension = 6.0

                self.construct_room(structures, (env_anchor_x,env_anchor_y), patch_size,
                                    (first_room_x_dimension,first_room_y_dimension), [[],[0,3],[],[]],[(int(-env_anchor_x/patch_size),int(-env_anchor_y/patch_size))])

                corridor_length = 4.5

                self.extend_mixed_corridor(structures, [env_anchor_x+first_room_x_dimension,env_anchor_y+patch_size/2.0,0,0,0,0], patch_size, corridor_length)

                self.extend_mixed_corridor(structures, [env_anchor_x+first_room_x_dimension,env_anchor_y+first_room_y_dimension-patch_size/2.0,0,0,0,0], patch_size, corridor_length)

                second_room_x_dimension = 4.5
                second_room_y_dimension = 6.0

                self.construct_room(structures, (env_anchor_x+first_room_x_dimension+corridor_length,env_anchor_y), patch_size,
                                    (second_room_x_dimension,second_room_y_dimension), [[],[],[],[0,3]])

            elif surface_source == 'two_stair_environment':

                second_room_height = random.random() * 0.5 + 1.0

                env_anchor_x = random.random() * (-1.0-(-3.5)) - 3.5
                env_anchor_y = random.random() * (-0.75-(-5)) - 5

                self.goal_x = env_anchor_x + random.random() * (12.5 - 9.0) + 9.0
                self.goal_y = env_anchor_y + random.random() * (5.25 - 0.75) + 0.75

                patch_size = 1.5

                while(abs(-env_anchor_x-1.5) <= or_robot.foot_h/2.0+0.05 or
                      abs(-env_anchor_x-3.0) <= or_robot.foot_h/2.0+0.05 or
                      abs(-env_anchor_y-1.5) <= or_robot.foot_w+0.07 or
                      abs(-env_anchor_y-3.0) <= or_robot.foot_w+0.07 or
                      abs(-env_anchor_y-4.5) <= or_robot.foot_w+0.07):
                    env_anchor_x = random.random() * (-1.0-(-3.5)) - 3.5
                    env_anchor_y = random.random() * (-0.75-(-5)) - 5

                first_room_x_dimension = 4.5
                first_room_y_dimension = 6.0

                self.construct_room(structures, (env_anchor_x,env_anchor_y,0), patch_size,
                                    (first_room_x_dimension,first_room_y_dimension), [[],[0,3],[],[]],[(int(-env_anchor_x/patch_size),int(-env_anchor_y/patch_size))])

                corridor_length = 4.5

                self.extend_mixed_stair(structures, [env_anchor_x+first_room_x_dimension,env_anchor_y+patch_size/2.0,0,0,0,0], stair_length=corridor_length,stair_width=1.5,stair_step_depth=0.5,stair_step_height=second_room_height/9.0,max_tilt_angle=20)
                self.extend_mixed_stair(structures, [env_anchor_x+first_room_x_dimension,env_anchor_y+first_room_y_dimension-patch_size/2.0,0,0,0,0], stair_length=corridor_length,stair_width=1.5,stair_step_depth=0.5,stair_step_height=second_room_height/9.0,max_tilt_angle=20)

                second_room_x_dimension = 4.5
                second_room_y_dimension = 6.0

                self.construct_room(structures, (env_anchor_x+first_room_x_dimension+corridor_length,env_anchor_y,second_room_height), patch_size,
                                    (second_room_x_dimension,second_room_y_dimension), [[],[],[],[0,3]])

            elif surface_source == 'flat_ground_environment':
                self.goal_x = 1.0
                self.goal_y = 0.0

                flat_surface_size = (8.0,3.0)
                self.pave_flat_surfaces(structures, [-flat_surface_size[0]/4.0,-flat_surface_size[1]/2.0,0,0,0,0], flat_surface_size)

                if self.env.GetViewer() is not None:
                    self.env.GetViewer().SetCamera([
                        [ 0, -1,  0, self.goal_x],
                        [-1,  0,  0, self.goal_y],
                        [ 0,  0, -1, 19.0],
                        [0., 0., 0., 1.]])


            for struct in structures:
                if struct.type == 'ground':
                    temp_init_p = struct.projection_global_frame(np.array([[0.0],[0.0],[or_robot.robot_z]]),np.array([[0.0],[0.0],[-1.0]]))
                    temp_goal_p = struct.projection_global_frame(np.array([[self.goal_x],[self.goal_y],[or_robot.robot_z]]),np.array([[0.0],[0.0],[-1.0]]))

                    if temp_init_p is not None and struct.inside_polygon(temp_init_p) and temp_init_p[2,0] >= self.init_z:
                        self.init_z = temp_init_p[2,0]
                        self.start_dist_to_boundary = struct.dist_to_boundary(np.array([[0.0],[0.0],[self.init_z]]))

                    if temp_goal_p is not None and struct.inside_polygon(temp_goal_p) and temp_goal_p[2,0] >= self.goal_z:
                        self.goal_z = temp_goal_p[2,0]
                        self.goal_dist_to_boundary = struct.dist_to_boundary(np.array([[self.goal_x],[self.goal_y],[self.goal_z]]))


            self.structures = structures

            # reset
            self.surface_list = None
            self.surface_mesh_list = None


    def crafting_quadrilateral_surface(self,vertices,surface_type,id):

        boundaries = [(0,1),(1,2),(2,3),(3,0)]

        trimesh_vertices = np.zeros((4,3),dtype=float)
        for i in range(len(vertices)):
            for j in range(3):
                trimesh_vertices[i,j] = vertices[i][j]

        trimesh_indices = np.array([[0,1,2],[0,2,3]])

        trimesh = rave.RaveCreateKinBody(self.env,'')
        trimesh.SetName('surface_'+str(id))
        trimesh.InitFromTrimesh(rave.TriMesh(trimesh_vertices,trimesh_indices),False)

        vertex_1 = np.array(vertices[0])
        vertex_2 = np.array(vertices[1])
        vertex_3 = np.array(vertices[2])

        normal_vector = np.cross(vertex_2-vertex_1,vertex_3-vertex_2)
        unit_normal_vector = normal_vector / np.linalg.norm(normal_vector)

        d = -np.dot(unit_normal_vector,vertex_1)

        surface_parameter = [unit_normal_vector[0],unit_normal_vector[1],unit_normal_vector[2],d]

        surface = trimesh_surface(id,surface_parameter,vertices,boundaries,trimesh,trimesh_vertices,trimesh_indices)
        surface.type = surface_type

        self.env.AddKinBody(surface.kinbody)
        DrawSurface(self.env,surface)

        return surface