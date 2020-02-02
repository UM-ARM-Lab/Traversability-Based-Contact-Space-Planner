import math

from config_parameter import *
from transformation_conversion import *

def contact_contact_dist(contact1,contact2):
	T1 = contact1.transform
	T2 = contact2.transform

	de = math.sqrt((T1[0,3]-T2[0,3])**2 + (T1[1,3]-T2[1,3])**2 + (T1[2,3]-T2[2,3])**2)

	q1 = SO3_to_quaternion(contact1.transform[0:3,0:3])
	q2 = SO3_to_quaternion(contact2.transform[0:3,0:3])

	do = 1 - abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])

	wr = 0.5

	return de + wr * do


def one_side_hausdorff(motion_plan1,motion_plan2):
	contact_plan1 = motion_plan1.contact_sequence
	contact_plan2 = motion_plan2.contact_sequence

	hausdorff_dist = 0.0
	for contact1 in contact_plan1:
		shortest_dist = 999.0
		for contact2 in contact_plan2:
			if contact1.manip == contact2.manip:
				dist = contact_contact_dist(contact1,contact2)

				if dist < shortest_dist:
					shortest_dist = dist

		if shortest_dist > hausdorff_dist:
			hausdorff_dist = shortest_dist

			if hausdorff_dist == 999.0:
				break

	return hausdorff_dist


def plan_to_plan_dist(motion_plan1,motion_plan2):
	# calculate hausdorff distance between contact plans
	return max(one_side_hausdorff(motion_plan1,motion_plan2),one_side_hausdorff(motion_plan2,motion_plan1))
