import numpy as np
import math
import IPython

from config_parameter import *
from transformation_conversion import *
from drawing_functions import *

weight_cr_orientation = 0.5
attractive_range = 0.3
attractive_orientation_range = 0.3

class contact_point:
    def __init__(self,p,n,group=0):
        self.position = p
        self.normal = n
        self.group = group # 0: foot contact, 1: hand contact

    def __eq__(self,other):
        return (self.position[0] == other.position[0] and self.position[1] == other.position[1] and self.position[2] == other.position[2] and
                self.normal[0] == other.normal[0] and self.normal[1] == other.normal[1] and self.normal[2] == other.normal[2])

    def __ne__(self, other):
        return not self.__eq__(other)


class contact_region:
    def __init__(self,p,n,r,group=0):
        self.position = p
        self.normal = n
        self.radius = r
        self.group = group

class contact:
    def __init__(self,transform,manip):
        self.transform = transform # SE(3)
        self.manip = manip
        self.validity = True


def contact_contact_region_euclidean_dist(contact,contact_region):
    return position_contact_region_euclidean_dist(contact.transform[0:3,3:4],contact_region)


def contact_contact_region_orientation_dist(contact,contact_region):
    if contact.manip == 'l_hand' or contact.manip == 'r_hand':
        contact_normal = contact.transform[0:3,0:1]
    elif contact.manip == 'l_foot' or contact.manip == 'r_foot':
        contact_normal = -contact.transform[0:3,2:3]

    return normal_contact_region_orientation_dist(contact_normal.T,contact_region)


def position_contact_region_euclidean_dist(position,contact_region,get_projection_vector=False):
    center_translation = position - contact_region.position

    z_signed_dist = np.dot(center_translation.T,contact_region.normal)[0,0]

    xy_dist = max(math.sqrt((np.linalg.norm(center_translation))**2 - z_signed_dist**2) - contact_region.radius, 0.0)

    euclidean_dist = math.hypot(xy_dist,z_signed_dist)

    if get_projection_vector:
        if xy_dist == 0:
            projection_unit_vector = contact_region.normal
        else:
            z_projection_vector = z_signed_dist * contact_region.normal
            projection_vector = (center_translation - z_projection_vector) * (xy_dist)/(xy_dist+contact_region.radius) + z_projection_vector
            projection_vector_norm = np.linalg.norm(projection_vector)

            if projection_vector_norm > 0.001:
                projection_unit_vector = projection_vector / projection_vector_norm
            else:
                projection_unit_vector = np.array([[0],[0],[0]])

        return (xy_dist,abs(z_signed_dist),euclidean_dist,projection_unit_vector)
    else:
        return (xy_dist,abs(z_signed_dist),euclidean_dist)


def normal_contact_region_orientation_dist(normal,contact_region):

    orientation_dist = 1 - np.dot(normal,contact_region.normal)[0,0]

    return orientation_dist


def contact_contact_region_dist(contact,contact_region):
    xy_dist,z_dist,euclidean_dist = contact_contact_region_euclidean_dist(contact,contact_region)
    orientation_dist = contact_contact_region_orientation_dist(contact,contact_region)

    return euclidean_dist + weight_cr_orientation * orientation_dist


def attractive_vector(contact,contact_regions):

    # project the contact to the surfaces, and see if there is a fit (find closest surface)
    # to generate attactive force from the surface, the contact and the contact region should be within a range of distance and orientation

    attractive_vector = np.array([[0,0,0]],dtype=float).T

    nearest_contact_dist = 9999

    for region in contact_regions:

        do = contact_contact_region_orientation_dist(contact,region)

        if do < attractive_orientation_range:

            dxy,dz,de = contact_contact_region_euclidean_dist(contact,region)
            if de < attractive_range:

                d = de + weight_cr_orientation * do

                if d < nearest_contact_dist:
                    if dxy == 0:
                        attractive_vector = (cm/attractive_range) * np.dot((region.position - contact.transform[0:3,3:4]).T,region.normal)[0,0] * region.normal
                    else:
                        attractive_vector = (cm/attractive_range) * (region.position - contact.transform[0:3,3:4])

                    nearest_contact_dist = d

    return attractive_vector, nearest_contact_dist


def contact_sequence_contact_regions_dist(contact_sequence,contact_regions):

    valid_contact_num = 0
    total_dist = 0.0

    for contact in contact_sequence:

        _, nearest_contact_dist = attractive_vector(contact,contact_regions)

        if nearest_contact_dist != 9999:
            valid_contact_num = valid_contact_num + 1
            total_dist = total_dist + nearest_contact_dist
        else:
            return 4999

    return total_dist/valid_contact_num
