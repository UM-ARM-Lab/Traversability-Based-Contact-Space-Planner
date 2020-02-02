import openravepy as rave
import math
import numpy as np

deg_to_rad = math.pi/180.0
rad_to_deg = 180.0/math.pi

def rpy_to_SO3(rpy):
    roll = rpy[0] * deg_to_rad
    pitch = rpy[1] * deg_to_rad
    yaw = rpy[2] * deg_to_rad
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    SO3 = np.array([[cp*cy,-cp*sy,sp],
                    [cr*sy+cy*sr*sp,cr*cy-sr*sp*sy,-cp*sr],
                    [sr*sy-cr*cy*sp,cy*sr+cr*sp*sy,cr*cp]],dtype=float)


    return SO3

def inverse_SO3(SO3):
    return SO3.T

def inverse_SE3(SE3):
    inverse_SE3 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],dtype=float)
    inverse_SE3[0:3,0:3] = SE3[0:3,0:3].T
    inverse_SE3[0:3,3:4] = -np.dot(SE3[0:3,0:3].T,SE3[0:3,3:4])
    return inverse_SE3

def xyzrpy_to_SE3(xyzrpy):
    x = xyzrpy[0]
    y = xyzrpy[1]
    z = xyzrpy[2]
    roll = xyzrpy[3]
    pitch = xyzrpy[4]
    yaw = xyzrpy[5]

    SE3 = np.zeros((4,4),dtype=float)
    SE3[0:3,0:3] = rpy_to_SO3(xyzrpy[3:6])
    SE3[0,3] = x
    SE3[1,3] = y
    SE3[2,3] = z
    SE3[3,3] = 1

    return SE3

def xyzrpy_to_inverse_SE3(xyzrpy):
    return inverse_SE3(xyzrpy_to_SE3(xyzrpy))

# be careful, this mapping is actually a one-to-many mapping and may be incorrect. Only use in locally relative rotation
def SO3_to_rpy(transform):
    epsilon = 0.0001
    if abs(transform[0,0]) > epsilon and abs(transform[2,2]) > epsilon:
        roll = math.atan2(-transform[1,2],transform[2,2]) * rad_to_deg # roll
        pitch = math.atan2(transform[0,2],math.sqrt(transform[1,2]**2+transform[2,2]**2)) * rad_to_deg # pitch
        yaw = math.atan2(-transform[0,1],transform[0,0]) * rad_to_deg # yaw

    elif abs(transform[0,0]) > epsilon and abs(transform[2,2]) <= epsilon:
        pitch = math.atan2(transform[0,2],math.sqrt(transform[1,2]**2+transform[2,2]**2)) * rad_to_deg # pitch
        yaw = math.atan2(-transform[0,1],transform[0,0]) * rad_to_deg # yaw

        yaw_rad = yaw * deg_to_rad

        if abs(math.cos(yaw_rad)) > abs(math.sin(yaw_rad)):
            if transform[2,1] / math.cos(yaw_rad) > 0:
                roll = 90
            else:
                roll = -90
        else:
            if transform[2,0] / math.sin(yaw_rad) > 0:
                roll = 90
            else:
                roll = -90

    elif abs(transform[0,0]) <= epsilon and abs(transform[2,2]) > epsilon:
        roll = math.atan2(-transform[1,2],transform[2,2]) * rad_to_deg # roll
        pitch = math.atan2(transform[0,2],math.sqrt(transform[1,2]**2+transform[2,2]**2)) * rad_to_deg # pitch

        roll_rad = roll * deg_to_rad

        if abs(math.cos(roll_rad)) > abs(math.sin(roll_rad)):
            if transform[1,0] / math.cos(roll_rad) > 0:
                yaw = 90
            else:
                yaw = -90
        else:
            if transform[2,0] / math.sin(roll_rad) > 0:
                yaw = 90
            else:
                yaw = -90

    elif abs(transform[0,0]) <= epsilon and abs(transform[2,2]) <= epsilon:
        cr_equal_0 = False
        cp_equal_0 = False
        cy_equal_0 = False

        if abs(transform[0,2])-1 <= epsilon:
            cp_equal_0 = True

            if transform[0,2] > 0:
                pitch = 90
            else:
                pitch = -90

            if abs(transform[1,0]) > epsilon: # cr*sy != 0
                roll = math.atan2(transform[2,0],transform[1,0]) * rad_to_deg # roll
                yaw = math.asin(max(min(transform[1,0]/math.cos(roll*deg_to_rad),1.0),-1.0)) * rad_to_deg # yaw
            else: # cr*sy == 0, sr*sy == +-1
                if pitch == 90:
                    if transform[2,0] < transform[1,1]:
                        roll = -90
                        yaw = 90
                    else:
                        roll = 90
                        yaw = 90
                else:
                    if transform[2,0] > 0:
                        roll = 90
                        yaw = 90
                    else:
                        roll = -90
                        yaw = 90

        else:
            cy_equal_0 = True
            cr_equal_0 = True

            pitch = -math.asin(transform[1,1]/transform[2,0]) * rad_to_deg
            roll = -math.asin(transform[1,2]/math.cos(pitch*deg_to_rad)) * rad_to_deg
            yaw = -math.asin(transform[0,1]/math.cos(pitch*deg_to_rad)) * rad_to_deg

    return [roll, pitch, yaw]

def SE3_to_xyzrpy(transform):
    x = round(transform[0,3],5) # x
    y = round(transform[1,3],5) # y
    z = round(transform[2,3],5) # z
    return [x,y,z] + SO3_to_rpy(transform)

# def SO3_to_quaternion(transform):
#     Qxx = transform[0,0]
#     Qyy = transform[1,1]
#     Qzz = transform[2,2]

#     Qxy = transform[0,1]
#     Qyx = transform[1,0]
#     Qxz = transform[0,2]
#     Qzx = transform[2,0]
#     Qyz = transform[1,2]
#     Qzy = transform[2,1]

#     t = Qxx + Qyy + Qzz
#     r = math.sqrt(1 + t)
#     w = 0.5 * r
#     x = sign(Qzy-Qyz) * 0.5 * math.sqrt(1+Qxx-Qyy-Qzz)
#     y = sign(Qxz-Qzx) * 0.5 * math.sqrt(1-Qxx+Qyy-Qzz)
#     z = sign(Qyx-Qxy) * 0.5 * math.sqrt(1-Qxx-Qyy+Qzz)

#     return [w,x,y,z]

def SO3_to_quaternion(transform):
    Qxx = transform[0,0]
    Qyy = transform[1,1]
    Qzz = transform[2,2]

    Qxy = transform[0,1]
    Qyx = transform[1,0]
    Qxz = transform[0,2]
    Qzx = transform[2,0]
    Qyz = transform[1,2]
    Qzy = transform[2,1]

    t = Qxx + Qyy + Qzz
    r = math.sqrt(1 + t)
    w = 0.5 * r
    x = 0.25 * (Qzy-Qyz) / w
    y = 0.25 * (Qxz-Qzx) / w
    z = 0.25 * (Qyx-Qxy) / w

    if w == 0:
        if x == 0:
            if y == 0:
                if z < 0:
                    z = -z;

            elif y < 0:
                y = -y; z = -z;

        elif x < 0:
            x = -x; y = -y; z = -z;

    return [w,x,y,z]

def quaternion_to_SO3(quat):
    quat_norm = np.linalg.norm(quat)

    w = quat[0]/quat_norm
    x = quat[1]/quat_norm
    y = quat[2]/quat_norm
    z = quat[3]/quat_norm

    R = np.array([[1-2*y**2-2*z**2, 2*x*y-2*z*w, 2*x*z+2*y*w],
                  [2*x*y+2*z*w, 1-2*x**2-2*z**2, 2*y*z-2*x*w],
                  [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x**2-2*y**2]])

    return R

def rotation_matrix_against_axis(axis,orientation):
    rotation_matrix = np.array([[1.0,0,0,0],[0,1.0,0,0],[0,0,1.0,0],[0,0,0,1.0]])
    ax = axis[0]; ay = axis[1]; az = axis[2]
    sin = math.sin(orientation); cos = math.cos(orientation)
    rotation_matrix[0,0] = cos+(ax**2)*(1-cos)
    rotation_matrix[0,1] = ax*ay*(1-cos)-az*sin
    rotation_matrix[0,2] = ax*az*(1-cos)+ay*sin
    rotation_matrix[1,0] = ay*az*(1-cos)+az*sin
    rotation_matrix[1,1] = cos+(ay**2)*(1-cos)
    rotation_matrix[1,2] = ay*az*(1-cos)-ax*sin
    rotation_matrix[2,0] = az*ax*(1-cos)-ay*sin
    rotation_matrix[2,1] = az*ay*(1-cos)+ax*sin
    rotation_matrix[2,2] = cos+(az**2)*(1-cos)
    # print(rotation_matrix)
    return rotation_matrix

def angle_conversion(angle):
    while angle >= 180.0 or angle < -180.0:
        if angle >= 180.0:
            angle = angle - 360
        elif angle < -180.0:
            angle = angle + 360

    return angle

def sign(input):
    if input >= 0:
        return 1
    else:
        return -1

def rotation_interpolation(r1,r2,t):
    quat1 = rave.quatFromRotationMatrix(r1)
    quat2 = rave.quatFromRotationMatrix(r2)
    # print(r1)
    # print(r2)
    t = min(max(t,0),1)

    quat_interpolate = rave.quatSlerp(quat1,quat2,t,True)
    return rave.matrixFromQuat(quat_interpolate)[0:3,0:3]

def first_terminal_angle(angle):
    # return the conterminal angle of angle1 that is within -180 to 180
    result_angle = angle
    while(result_angle < -180 or result_angle >= 180):
        if result_angle < -180:
            result_angle = result_angle + 360
        elif result_angle >= 180:
            result_angle = result_angle - 360

    return result_angle

def angle_diff(angle1,angle2):
    # returns angle1-angle2 with minimum abs
    diff = angle1-angle2

    return first_terminal_angle(diff)

def angle_mean(angle1,angle2):
    # return the closest mean angle in -180 ~ 180
    mean1 = (angle1+angle2)/2.0
    mean2 = (angle1+angle2)/2.0 + 180

    # print('angle1: %5.3f, angle2: %5.3f.'%(angle1,angle2))
    # print('mean1: %5.3f, mean2: %5.3f.'%(mean1,mean2))
    # print('diff1: %5.3f, diff2: %5.3f.'%(angle_diff(mean1,angle1),angle_diff(mean2,angle1)))

    if abs(angle_diff(mean1,angle1)) < abs(angle_diff(mean2,angle1)):
        return mean1
    else:
        return mean2

def list_mean(input_list):
    if len(input_list) != 0:
        return sum(input_list)/float(len(input_list))
    else:
        raw_input('ERROR: Calculating the mean of an empty list.')
        return None

def mean(v1,v2,v3=None,v4=None,v5=None,v6=None):
    if v3 is None:
        return (v1+v2)/2.0
    elif v4 is None:
        return (v1+v2+v3)/3.0
    elif v5 is None:
        return (v1+v2+v3+v4)/4.0
    elif v6 is None:
        return (v1+v2+v3+v4+v5)/5.0
    else:
        return (v1+v2+v3+v4+v5+v6)/6.0

def gaussian_function(x,mean,cov):
    degree = np.shape(x)[0]
    diff = x-mean
    cov_det = np.linalg.det(cov)
    cov_inv = np.linalg.inv(cov)

    return math.exp(-0.5 * np.dot(np.dot(diff.T,cov_inv),diff)) / (math.sqrt(cov_det*(2*math.pi)**degree))

def SE3_translation_distance(transform1,transform2):
    return np.linalg.norm(transform1[0:3,3]-transform2[0:3,3])

def quat_orientation_distance(quat1,quat2):
    return 1 - abs(np.dot(quat1,quat2))
