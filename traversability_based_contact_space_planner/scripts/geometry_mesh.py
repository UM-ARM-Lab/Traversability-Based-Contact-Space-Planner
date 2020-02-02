import numpy as np
import math

import openravepy as rave

from transformation_conversion import deg_to_rad

def openrave_cylinder(env,name,z_range,r):
    infocylinder = rave.KinBody.GeometryInfo()
    infocylinder._type = rave.GeometryType.Cylinder
    infocylinder._t[0,3] = 0.0
    infocylinder._t[1,3] = 0.0
    infocylinder._t[2,3] = (z_range[0] + z_range[1])/2.0
    infocylinder._vGeomData = [r,(z_range[1]-z_range[0])]
    infocylinder._bVisible = False
    infocylinder._fTransparency = 0.0
    infocylinder._vDiffuseColor = [1,0,0]

    cylinder = rave.RaveCreateKinBody(env,'')
    cylinder.InitFromGeometries([infocylinder])
    cylinder.SetName(name)

    return cylinder

def trimesh_cylinder(env,name,z_range,r):
    trimesh_cylinder = rave.RaveCreateKinBody(env,'')
    trimesh_cylinder.SetName(name)
    check_range_z = z_range
    check_range_r = r

    circle_divide = 36
    cylinder_trimesh_vertices = np.zeros((circle_divide*2+2,3))
    cylinder_trimesh_indices = np.zeros((circle_divide*4,3))

    for t in range(circle_divide*2):
        if t < circle_divide:
            cz = check_range_z[0]
        else:
            cz = check_range_z[1]

        cos_theta = math.cos((t%circle_divide)*(360.0/circle_divide)*deg_to_rad)
        sin_theta = math.sin((t%circle_divide)*(360.0/circle_divide)*deg_to_rad)
        cx = check_range_r * cos_theta
        cy = check_range_r * sin_theta
        cylinder_trimesh_vertices[t:t+1,0:3] = np.array((cx,cy,cz))

    cylinder_trimesh_vertices[circle_divide*2,0:3] = np.array((0,0,check_range_z[0]))
    cylinder_trimesh_vertices[circle_divide*2+1,0:3] = np.array((0,0,check_range_z[1]))


    for t in range(circle_divide*2):
        if t < circle_divide:
            center_index = circle_divide*2
            vertex1 = t % circle_divide
            vertex2 = (t+1) % circle_divide
            vertex3 = t % circle_divide + circle_divide
            cylinder_trimesh_indices[2*t:2*t+1,0:3] = np.array([center_index,vertex2,vertex1])
            cylinder_trimesh_indices[2*t+1:2*(t+1),0:3] = np.array([vertex1,vertex2,vertex3])

        else:
            center_index = circle_divide*2+1
            vertex1 = t % circle_divide + circle_divide
            vertex2 = (t+1) % circle_divide + circle_divide
            vertex3 = (t+1) % circle_divide
            cylinder_trimesh_indices[2*t:2*t+1,0:3] = np.array([center_index,vertex1,vertex2])
            cylinder_trimesh_indices[2*t+1:2*(t+1),0:3] = np.array([vertex1,vertex3,vertex2])

    trimesh_cylinder.InitFromTrimesh(rave.TriMesh(cylinder_trimesh_vertices,cylinder_trimesh_indices),False)

    return trimesh_cylinder

def trimesh_fan(env,name,z_range,angle_range,r,origin=(0,0)):
    min_angle = angle_range[0]
    max_angle = angle_range[1]
    trimesh_fan = rave.RaveCreateKinBody(env,'')
    trimesh_fan.SetName(name)
    check_range_z = z_range
    check_range_r = r

    arc_vertices_num = (max_angle-min_angle)/10 + 1
    fan_trimesh_vertices = np.zeros((arc_vertices_num*2+2,3))
    fan_trimesh_indices = np.zeros(((arc_vertices_num-1)*4+4,3))

    for t in range(arc_vertices_num*2):
        if t < arc_vertices_num:
            cz = check_range_z[0]
        else:
            cz = check_range_z[1]

        cos_theta = math.cos(((t%arc_vertices_num)*10+min_angle)*deg_to_rad)
        sin_theta = math.sin(((t%arc_vertices_num)*10+min_angle)*deg_to_rad)
        cx = check_range_r * cos_theta + origin[0]
        cy = check_range_r * sin_theta + origin[1]
        fan_trimesh_vertices[t:t+1,0:3] = np.array((cx,cy,cz))

    fan_trimesh_vertices[arc_vertices_num*2,0:3] = np.array((origin[0],origin[1],check_range_z[0]))
    fan_trimesh_vertices[arc_vertices_num*2+1,0:3] = np.array((origin[0],origin[1],check_range_z[1]))

    lower_center_index = arc_vertices_num*2
    upper_center_index = arc_vertices_num*2+1

    for t in range(arc_vertices_num*2):
        if t < arc_vertices_num-1:
            vertex1 = t % arc_vertices_num
            vertex2 = (t+1) % arc_vertices_num
            vertex3 = t % arc_vertices_num + arc_vertices_num
            fan_trimesh_indices[2*t:2*t+1,0:3] = np.array([lower_center_index,vertex2,vertex1])
            fan_trimesh_indices[2*t+1:2*t+2,0:3] = np.array([vertex1,vertex2,vertex3])

        elif t > arc_vertices_num-1 and t < 2*arc_vertices_num-1:
            vertex1 = t % arc_vertices_num + arc_vertices_num
            vertex2 = (t+1) % arc_vertices_num + arc_vertices_num
            vertex3 = (t+1) % arc_vertices_num
            fan_trimesh_indices[2*t-2:2*t-1,0:3] = np.array([upper_center_index,vertex1,vertex2])
            fan_trimesh_indices[2*t-1:2*t,0:3] = np.array([vertex1,vertex3,vertex2])

    fan_trimesh_indices[(arc_vertices_num-1)*4:(arc_vertices_num-1)*4+1,0:3] = np.array([lower_center_index,0,arc_vertices_num])
    fan_trimesh_indices[(arc_vertices_num-1)*4+1:(arc_vertices_num-1)*4+2,0:3] = np.array([lower_center_index,arc_vertices_num,upper_center_index])
    fan_trimesh_indices[(arc_vertices_num-1)*4+2:(arc_vertices_num-1)*4+3,0:3] = np.array([upper_center_index,2*arc_vertices_num-1,arc_vertices_num-1])
    fan_trimesh_indices[(arc_vertices_num-1)*4+3:(arc_vertices_num-1)*4+4,0:3] = np.array([upper_center_index,arc_vertices_num-1,lower_center_index])

    trimesh_fan.InitFromTrimesh(rave.TriMesh(fan_trimesh_vertices,fan_trimesh_indices),False)
    trimesh_fan.GetLinks()[0].GetGeometries()[0].SetTransparency(0.2)

    return trimesh_fan