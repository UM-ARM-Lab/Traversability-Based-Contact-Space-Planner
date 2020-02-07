import numpy as np
import math
import openravepy as rave
import copy
import random
import time
import IPython

from config_parameter import *
from transformation_conversion import *

hiwa = [190,194,63]
midori = [34,125,81]
hiwamoegi = [144,180,75]
tamago = [249,191,69]
urayanagi = [181,202,160]
nae = [134,193,102]
aotake = [0,137,108]
kihada = [251,226,81]
ominaeshi = [221,210,59]
tokiwa = [27,129,62]
color_library = [hiwa, midori, hiwamoegi, tamago, urayanagi, nae, aotake, kihada, ominaeshi, tokiwa]


def DrawPaths(current,env,handles=None):

    if(handles == None):
        handles = draw_handles

    angle_range = 120.0
    c = current
    while c.parent is not None:
        r = 0; g = 0; b = 0

        if(c.base[5] >= 0):
            g = (angle_range - c.base[5]) / angle_range
            r = c.base[5] / angle_range
            b = 0
        else:
            g = -(-angle_range - c.base[5]) / angle_range
            b = -c.base[5] / angle_range
            r = 0

        r = max(min(r,1),0)
        g = max(min(g,1),0)
        b = max(min(b,1),0)

        base_trajectory = np.array([[c.parent.base[0], c.parent.base[1], c.parent.base[2]],
                                    [c.base[0], c.base[1], c.base[2]]])

        handles.append(env.drawlinestrip(points = base_trajectory, linewidth = 5.0, colors = np.array(((r,g,b),(r,g,b)))))
        # handles.append(env.drawlinestrip(points = base_trajectory, linewidth = 5.0, colors = np.array(((0,1,0),(0,1,0)))))

        c = c.parent


def DrawGridPath(grid_path,env,torso_pose_grid,color,z=0.2,size_ratio=1.0,handles=None,y_offset=0,style='single'):

    arrow_length = 0.1
    if(handles is None):
        handles = draw_handles

    for i,node in enumerate(grid_path):
        orientation = grid_path[i][2] * deg_to_rad
        from_vec = np.array([[grid_path[i][0]],[grid_path[i][1]+y_offset],[z]])
        to_vec = np.array([[grid_path[i][0]+arrow_length*math.cos(orientation)],[grid_path[i][1]+arrow_length*math.sin(orientation)+y_offset],[z]])
        handles.append(env.drawarrow(from_vec , to_vec, 0.005, [color[0], color[1], color[2]]))

        handles.append(env.plot3(from_vec.T,
                                  pointsize=0.05*size_ratio,
                                  colors=(color[0],color[1],color[2],0.7),
                                  drawstyle=1))

        if(i != 0):

            if style == 'single':
                base_trajectory = np.array([[grid_path[i-1][0], grid_path[i-1][1]+y_offset, z],
                                        [grid_path[i][0], grid_path[i][1]+y_offset, z]])

                handles.append(env.drawlinestrip(points = base_trajectory, linewidth = 5.0*size_ratio, colors = np.array((color,color))))
            elif style == 'double':
                direction_vec = np.array([grid_path[i][0] - grid_path[i-1][0], grid_path[i][1] - grid_path[i-1][1]])
                direction_vec = direction_vec / np.linalg.norm(direction_vec)

                split_dist = 0.1


                base_trajectory_1 = np.array([[grid_path[i-1][0] + direction_vec[1]*split_dist/2.0, grid_path[i-1][1] + y_offset - direction_vec[0]*split_dist/2.0, z],
                                              [grid_path[i][0] + direction_vec[1]*split_dist/2.0, grid_path[i][1] + y_offset - direction_vec[0]*split_dist/2.0, z]])

                base_trajectory_2 = np.array([[grid_path[i-1][0] - direction_vec[1]*split_dist/2.0, grid_path[i-1][1] + y_offset + direction_vec[0]*split_dist/2.0, z],
                                              [grid_path[i][0] - direction_vec[1]*split_dist/2.0, grid_path[i][1] + y_offset + direction_vec[0]*split_dist/2.0, z]])

                handles.append(env.drawlinestrip(points = base_trajectory_1, linewidth = 4.0*size_ratio, colors = np.array((color,color))))
                handles.append(env.drawlinestrip(points = base_trajectory_2, linewidth = 4.0*size_ratio, colors = np.array((color,color))))


def DrawStances(current,or_robot,env,handles=None,marker='normal'):

    if(handles == None):
        handles = draw_handles

    foot_h = or_robot.foot_h
    foot_w = or_robot.foot_w
    hand_h = or_robot.hand_h
    hand_w = or_robot.hand_w

    if marker == 'normal':
        foot_corners = np.array([[foot_h/2, foot_w/2, 0.01, 1],
                                [-foot_h/2, foot_w/2, 0.01, 1],
                                [-foot_h/2, -foot_w/2, 0.01, 1],
                                [foot_h/2, -foot_w/2, 0.01, 1]])
        foot_corners = np.transpose(foot_corners)

        hand_corners = np.array([[-0.01, hand_h/2, hand_w/2, 1],
                                [-0.01, -hand_h/2, hand_w/2, 1],
                                [-0.01, -hand_h/2, -hand_w/2, 1],
                                [-0.01, hand_h/2, -hand_w/2, 1]])
        hand_corners = np.transpose(hand_corners)

    elif marker == 'cavalry':

        foot_corners = np.array([[foot_h/2, foot_w/2, 0.01, 1],
                                [-foot_h/2, foot_w/2, 0.01, 1],
                                [-foot_h/2, -foot_w/2, 0.01, 1],
                                [foot_h/2, -foot_w/2, 0.01, 1],
                                [foot_h/2, foot_w/2, 0.01, 1],
                                [-foot_h/2, -foot_w/2, 0.01, 1]])
        foot_corners = np.transpose(foot_corners)

        hand_corners = np.array([[-0.01, hand_h/2, hand_w/2, 1],
                                [-0.01, -hand_h/2, hand_w/2, 1],
                                [-0.01, -hand_h/2, -hand_w/2, 1],
                                [-0.01, hand_h/2, -hand_w/2, 1],
                                [-0.01, hand_h/2, hand_w/2, 1],
                                [-0.01, -hand_h/2, -hand_w/2, 1]])
        hand_corners = np.transpose(hand_corners)

    c = current
    while c is not None:
        # draw left foot pose
        if isinstance(c.left_leg, list):
            left_leg_transform = xyzrpy_to_SE3(c.left_leg)
        elif isinstance(c.left_leg, np.ndarray):
            left_leg_transform = c.left_leg

        foot_corners_transformed = np.dot(left_leg_transform,foot_corners)
        foot_corners_transformed = np.delete(foot_corners_transformed, 3, 0)
        foot_corners_transformed = np.transpose(foot_corners_transformed)
        foot_corners_transformed = np.append(foot_corners_transformed, [foot_corners_transformed[0,:]], 0)
        handles.append(env.drawlinestrip(points = foot_corners_transformed,
        linewidth = 5.0,
        colors = np.array([(1,0,0)]*(foot_corners.shape[1]+1))))

        # draw right foot pose

        if isinstance(c.right_leg, list):
            right_leg_transform = xyzrpy_to_SE3(c.right_leg)
        elif isinstance(c.right_leg, np.ndarray):
            right_leg_transform = c.right_leg

        foot_corners_transformed = np.dot(right_leg_transform,foot_corners)
        foot_corners_transformed = np.delete(foot_corners_transformed, 3, 0)
        foot_corners_transformed = np.transpose(foot_corners_transformed)
        foot_corners_transformed = np.append(foot_corners_transformed, [foot_corners_transformed[0,:]], 0)
        handles.append(env.drawlinestrip(points = foot_corners_transformed,
        linewidth = 5.0,
        colors = np.array([(0,1,0)]*(foot_corners.shape[1]+1))))

        # draw left hand pose
        if((isinstance(c.left_arm, list) and c.left_arm[0] != -99.0) or
           (isinstance(c.left_arm, np.ndarray) and c.left_arm is not None)):

            if isinstance(c.left_arm, list):
                left_arm_transform = xyzrpy_to_SE3(c.left_arm)
            elif isinstance(c.left_leg, np.ndarray):
                left_arm_transform = c.left_arm

            hand_corners_transformed = np.dot(left_arm_transform,hand_corners)
            hand_corners_transformed = np.delete(hand_corners_transformed, 3, 0)
            hand_corners_transformed = np.transpose(hand_corners_transformed)
            hand_corners_transformed = np.append(hand_corners_transformed, [hand_corners_transformed[0,:]], 0)
            handles.append(env.drawlinestrip(points = hand_corners_transformed,
            linewidth = 5.0,
            colors = np.array([(0,0,1)]*(hand_corners.shape[1]+1))))

        # draw right hand pose
        if((isinstance(c.right_arm, list) and c.right_arm[0] != -99.0) or
           (isinstance(c.right_arm, np.ndarray) and c.right_arm is not None)):

            if isinstance(c.right_arm, list):
                right_arm_transform = xyzrpy_to_SE3(c.right_arm)
            elif isinstance(c.right_leg, np.ndarray):
                right_arm_transform = c.right_arm

            hand_corners_transformed = np.dot(right_arm_transform,hand_corners)
            hand_corners_transformed = np.delete(hand_corners_transformed, 3, 0)
            hand_corners_transformed = np.transpose(hand_corners_transformed)
            hand_corners_transformed = np.append(hand_corners_transformed, [hand_corners_transformed[0,:]], 0)
            handles.append(env.drawlinestrip(points = hand_corners_transformed,
            linewidth = 5.0,
            colors = np.array([(1,1,0)]*(hand_corners.shape[1]+1))))

        if isinstance(c.parent,tuple):
            c = c.parent[0]
        else:
            c = c.parent


def DrawPoints(env,positions,rgb=(0,0,0,1),size=0.2,handles=None,drawstyle=1):

    if(handles == None):
        handles = draw_handles

    for position in positions:
        DrawPoint(env,position,rgb=rgb,size=size,handles=handles,drawstyle=drawstyle)


def DrawPoint(env,position,rgb=(0,0,0,1),size=0.2,handles=None,drawstyle=1):

    if(handles == None):
        handles = draw_handles

    input_position = np.array(position)

    handles.append(env.plot3(input_position, pointsize=size, colors=rgb, drawstyle=drawstyle))
    # handles.append(env.plot3(input_position, pointsize=0.01, colors=rgb, drawstyle=1))


def DrawLocation(env, transform, rgb=[0,0,0]):
    if (np.shape(transform) == (4,4)):
        draw_handles.append(env.plot3(transform[0:3,3],
                                      pointsize=0.2,
                                      colors=rgb,
                                      drawstyle=1))
        draw_handles.append(env.plot3(transform[0:3,3],
                                      pointsize=15.0,
                                      colors=rgb))
    elif (np.shape(transform) == (4,)):
        draw_handles.append(env.plot3(transform[0:3],
                                      pointsize=15.0,
                                      colors=rgb))
    elif (np.shape(transform) == (3,)):
        draw_handles.append(env.plot3(transform,
                                      pointsize=15.0,
                                      colors=rgb))


def DrawOrientation(env, transform, size=0.2, handles=None):

    if(handles == None):
        handles = draw_handles

    if (np.shape(transform) == (4,4) or np.shape(transform) == (3,3)):
        from_vec = []
        # if (location == None):
        from_vec = transform[0:3,3]
        # elif (type(location) == geometry_msgs.msg._Point.Point):
        #     from_vec = self.utils.PointToArray(location.position)

        to_vec_1 = from_vec + size*(transform[0:3,0])
        to_vec_2 = from_vec + size*(transform[0:3,1])
        to_vec_3 = from_vec + size*(transform[0:3,2])

        handles.append(env.drawarrow(from_vec, to_vec_1, 0.005, [1, 0, 0]))
        handles.append(env.drawarrow(from_vec, to_vec_2, 0.005, [0, 1, 0]))
        handles.append(env.drawarrow(from_vec, to_vec_3, 0.005, [0, 0, 1]))


def DrawRegion(env,transform,radius):
	center = transform[0:3,3:4]
	x_vector = transform[0:3,0:1]
	y_vector = transform[0:3,1:2]

	region_boundary_points = np.zeros((37,3),dtype=float)

	for i in range(37):
		region_boundary_point = center + x_vector * radius * math.cos(i*10*deg_to_rad) + y_vector * radius * math.sin(i*10*deg_to_rad)
		region_boundary_points[i:i+1,:] = region_boundary_point.T

	draw_handles.append(env.drawlinestrip(points = region_boundary_points,linewidth = 5.0,colors = np.array((0,0,0))))

	arrow_points = np.zeros((5,3),dtype=float)

	arrow_points[0:1,:] = (center - radius*2.0/3.0 * x_vector).T
	arrow_points[1:2,:] = (center + radius*2.0/3.0 * x_vector).T
	arrow_points[2:3,:] = (center + radius/2.0 * y_vector).T
	arrow_points[3:4,:] = (center + radius*2.0/3.0 * x_vector).T
	arrow_points[4:5,:] = (center - radius/2.0 * y_vector).T

	draw_handles.append(env.drawlinestrip(points = arrow_points,linewidth = 5.0,colors = np.array((0,0,0))))


def DrawContactRegion(env,center,normal,radius):

    if(normal[0,0] == 0 and normal[1,0] == 0):
        x_vector = np.array([[1.0],[0],[0]],dtype=float)
    else:
        x_vector = np.array([[normal[1,0]],[-normal[0,0]],[0]],dtype=float)
        x_vector = 1/np.linalg.norm(x_vector) * x_vector
    y_vector = np.cross(normal.T,x_vector.T).T

    region_boundary_points = np.zeros((37,3),dtype=float)

    for i in range(37):
        region_boundary_point = center + x_vector * radius * math.cos(i*10*deg_to_rad) + y_vector * radius * math.sin(i*10*deg_to_rad)
        region_boundary_points[i:i+1,:] = region_boundary_point.T

    draw_handles.append(env.drawlinestrip(points = region_boundary_points,linewidth = 4.0,colors = np.array((0,0,0))))

    draw_handles.append(env.drawarrow(center, center + 0.1 * normal, 0.005, [1, 0, 0]))


def DrawLineStrips(env,from_vec,to_vec,color=(0,0,0),linewidth=5.0):
    line_endpoints = np.zeros((2,3),dtype=float)
    line_endpoints[0:1,0:3] = np.array(from_vec)
    line_endpoints[1:2,0:3] = np.array(to_vec)

    draw_handles.append(env.drawlinestrip(points = line_endpoints,linewidth = linewidth,colors = np.array(color)))


def DrawSurface(env,struct,transparency=1.0,style='greyscale'):

    if style == 'random_color':
        r = random.random()
        g = random.random()
        b = random.random()

        total_rgb = math.sqrt(r**2+g**2+b**2)
        r = r/total_rgb
        g = g/total_rgb
        b = b/total_rgb

    elif style == 'greyscale':
        greyscale = random.random() * 0.6 + 0.2

        r = greyscale
        g = greyscale
        b = greyscale

    elif style == 'random_green_yellow_color':
        color = random.choice(color_library)
        r = color[0] / 255.0
        g = color[1] / 255.0
        b = color[2] / 255.0

    # DrawOrientation(env, struct.transform)

    for boundary in struct.boundaries:

		boundaries_point = np.zeros((2,3),dtype=float)
		boundaries_point[0:1,:] = np.atleast_2d(np.array(struct.vertices[boundary[0]]))
		boundaries_point[1:2,:] = np.atleast_2d(np.array(struct.vertices[boundary[1]]))

		draw_handles.append(env.drawlinestrip(points = boundaries_point,linewidth = 5.0,colors = np.array((r,g,b))))

    draw_handles.append(env.drawtrimesh(struct.kinbody.GetLinks()[0].GetCollisionData().vertices,struct.kinbody.GetLinks()[0].GetCollisionData().indices,colors=np.array([r,g,b,transparency])))


def DrawCells(env,cell_indices_set,torso_pose_grid):
    cell_indices_list = list(cell_indices_set)

    for cell_indices in cell_indices_list:
        position = torso_pose_grid.grid_to_position(cell_indices)

        lines_points = np.zeros((2,3),dtype=float)
        lines_points[0,:] = np.array([position[0],position[1],0.2])
        lines_points[1,:] = np.array([position[0]+0.05*math.cos(position[2]*deg_to_rad),position[1]+0.05*math.sin(position[2]*deg_to_rad),0.2])

        draw_handles.append(env.drawlinestrip(points = lines_points,linewidth = 2.0,colors = np.array((1,0,0))))


def RefreshHandler():
    while draw_handles:
        del draw_handles[-1]

    while contact_draw_handles:
        del contact_draw_handles[-1]
