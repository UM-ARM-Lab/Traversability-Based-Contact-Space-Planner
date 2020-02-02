# 3rd-Party Imports
import numpy as np
import math
import openravepy as rave
import copy
import IPython

# Local Imports
from cbirrtpy import CBiRRT

from config_parameter import *
from transformation_conversion import *


def parabola_traj_fitting(m1,m2,init_vertical_translation,parabola_lifting_dist):
	# z = a*(xy-xy0)**2 + b

	xy1 = 0
	z1 = 0 + init_vertical_translation
	xy2 = math.sqrt((m2[0]-m1[0])**2 + (m2[1]-m1[1])**2)
	z2 = m2[2]-m1[2] + init_vertical_translation

	b = max(z1,z2) + parabola_lifting_dist

	if(abs(z1 - z2) < 0.0001):
		xy0 = (xy1+xy2)/2.0
	else:
		K = (z1-z2)/(xy1-xy2)
		xy0 = xy1 - (z1-b)/K + math.sqrt((xy1-(z1-b)/K)**2 - (xy1**2 - (xy1+xy2)*(z1-b)/K))

	a = (z2-b)/(xy2-xy0)**2
	b = b + m1[2]

	return (xy0,a,b)


def hand_parabola_traj_fitting(m1,m2):
	# z = a*(xy-xy0)**2 + b
	init_vertical_translation = 0.06
	parabola_lifting_dist = 0.1

	xy1 = 0
	z1 = 0 + init_vertical_translation
	xy2 = math.sqrt((m2[0]-m1[0])**2 + (m2[1]-m1[1])**2)
	z2 = m2[2]-m1[2] + init_vertical_translation

	b = max(z1,z2) + parabola_lifting_dist

	if(abs(z1 - z2) < 0.0001):
		xy0 = (xy1+xy2)/2.0
	else:
		K = (z1-z2)/(xy1-xy2)
		print('z1:%5.2f, z2:%5.2f, K:%5.2f'%(z1,z2,K))
		xy0 = xy1 - (z1-b)/K + math.sqrt((xy1-(z1-b)/K)**2 - (xy1**2 - (xy1+xy2)*(z1-b)/K))

	a = (z2-b)/(xy2-xy0)**2
	b = b + m1[2]

	return (xy0,a,b)

def transition_manipulator(general_ik_interface,or_robot,current_state,m1,m2,moving_manipulator,interpolate_step,traj,plan_waypoint_contact_manips,prev_contact_manip_group,manip_group_num):

	env = or_robot.env
	robot = or_robot.robot
	DOFNameIndexDict = or_robot.DOFNameIndexDict
	DOFNameActiveIndexDict = or_robot.DOFNameActiveIndexDict
	lower_limits = or_robot.lower_limits
	higher_limits = or_robot.higher_limits
	IKInitDOFValues = or_robot.IKInitDOFValues
	StandingPostureDOFValues = or_robot.StandingPostureDOFValues
	robot_z = or_robot.robot_z

	com_start = robot.GetCenterOfMass().tolist()

	ll = current_state.left_leg
	rl = current_state.right_leg
	la = current_state.left_arm
	ra = current_state.right_arm

	manip_poses = [ll,rl,la,ra]

	l_arm_indices = robot.GetManipulator('l_arm').GetArmIndices()
	r_arm_indices = robot.GetManipulator('r_arm').GetArmIndices()
	l_leg_indices = robot.GetManipulator('l_leg').GetArmIndices()
	r_leg_indices = robot.GetManipulator('r_leg').GetArmIndices()

	additional_active_DOF_indices = [None]*len(or_robot.additional_active_DOFs)
	for index,j in enumerate(or_robot.additional_active_DOFs):
		additional_active_DOF_indices[index] = robot.GetJoint(j).GetDOFIndex()

	whole_body_indices = np.concatenate((l_arm_indices, r_arm_indices, l_leg_indices, r_leg_indices, additional_active_DOF_indices))

	if(moving_manipulator == 'l_leg' or moving_manipulator == 'r_leg'):
		if(la[0] == -99.0 and ra[0] == -99.0):
			active_dof_indices = np.concatenate((l_leg_indices, r_leg_indices, additional_active_DOF_indices))
		elif(la[0] != -99.0 and ra[0] == -99.0):
			active_dof_indices = np.concatenate((l_arm_indices, l_leg_indices, r_leg_indices, additional_active_DOF_indices))
		elif(la[0] == -99.0 and ra[0] != -99.0):
			active_dof_indices = np.concatenate((r_arm_indices, l_leg_indices, r_leg_indices, additional_active_DOF_indices))
		else:
			active_dof_indices = whole_body_indices

	elif(moving_manipulator == 'l_arm'):
		if(ra[0] == -99.0):
			active_dof_indices = np.concatenate((l_arm_indices, l_leg_indices, r_leg_indices, additional_active_DOF_indices))
		else:
			active_dof_indices = whole_body_indices

	elif(moving_manipulator == 'r_arm'):
		if(la[0] == -99.0):
			active_dof_indices = np.concatenate((r_arm_indices, l_leg_indices, r_leg_indices, additional_active_DOF_indices))
		else:
			active_dof_indices = whole_body_indices


	support_state_mapping = {0:'l_leg',1:'r_leg',2:'l_arm',3:'r_arm'}

	# decide manipulator trajectory parameters
	if(moving_manipulator == 'l_leg' or moving_manipulator == 'r_leg'):

		init_vertical_translation = 0.11
		parabola_lifting_dist = 0.02

		if(moving_manipulator == 'l_leg'):
			support_state = [False,True,la[0]!=-99.0,ra[0]!=-99.0]
			desired_com = [(current_state.left_leg[0]+2.0*current_state.right_leg[0])/3.0, (current_state.left_leg[1]+2.0*current_state.right_leg[1])/3.0, (current_state.left_leg[2]+2.0*current_state.right_leg[2])/3.0 + robot_z]
			s1 = rl
		elif(moving_manipulator == 'r_leg'):
			support_state = [True,False,current_state.left_arm[0]!=-99.0,current_state.right_arm[0]!=-99.0]
			desired_com = [(2.0*current_state.left_leg[0]+current_state.right_leg[0])/3.0, (2.0*current_state.left_leg[1]+current_state.right_leg[1])/3.0, (2.0*current_state.left_leg[2]+current_state.right_leg[2])/3.0 + robot_z]
			s1 = ll

		moving_vector = np.array([m2[0]-m1[0],m2[1]-m1[1]])
		foot_xy_escape_vector = None
		if np.linalg.norm(moving_vector) != 0:
			moving_vector = moving_vector / np.linalg.norm(moving_vector)
			inter_feet_vector = np.array([s1[0]-m1[0],s1[1]-m1[1]])
			foot_xy_escape_vector = -(inter_feet_vector - np.dot(inter_feet_vector,moving_vector) * moving_vector)

			if np.linalg.norm(foot_xy_escape_vector) != 0:
				foot_xy_escape_vector = foot_xy_escape_vector / np.linalg.norm(foot_xy_escape_vector)
			else:
				foot_xy_escape_vector = None

		x_dist = m2[0]-m1[0]
		y_dist = m2[1]-m1[1]

		first_z_translation = init_vertical_translation
		second_z_translation = init_vertical_translation
		xy_translation = 0.0

		if(math.sqrt(x_dist**2 + y_dist**2) > 0.001): # translation != 0
			if(abs(m1[2] - m2[2]) < 0.001): # the height(z) of the moving manipulator before and after moving is the same
				(xy0,a,b) = parabola_traj_fitting(m1,m2,init_vertical_translation,parabola_lifting_dist)
			else:
				if(m1[2] < m2[2]):
					first_z_translation = first_z_translation + (m2[2]-m1[2])/2.0
					new_m1 = copy.copy(m1)
					new_m1[2] = new_m1[2] + abs(m2[2]-m1[2])/2.0
					(xy0,a,b) = parabola_traj_fitting(new_m1,m2,init_vertical_translation,parabola_lifting_dist)
				else:
					second_z_translation = second_z_translation + (m2[2]-m1[2])/2.0
					new_m2 = copy.copy(m2)
					new_m2[2] = new_m2[2] + abs(m2[2]-m1[2])/2.0
					(xy0,a,b) = parabola_traj_fitting(m1,new_m2,init_vertical_translation,parabola_lifting_dist)

			step_dist = math.sqrt(x_dist**2 + y_dist**2) + first_z_translation + second_z_translation
			xy_translation = math.sqrt(x_dist**2 + y_dist**2)
		else:
			step_dist = first_z_translation + second_z_translation

	elif(moving_manipulator == 'l_arm' or moving_manipulator == 'r_arm'):
		init_vertical_translation = 0.06
		parabola_lifting_dist = 0.10

		desired_com = [(current_state.left_leg[0]+current_state.right_leg[0])/2.0, (current_state.left_leg[1]+current_state.right_leg[1])/2.0, (current_state.left_leg[2]+current_state.right_leg[2])/2.0 + robot_z]

		if(moving_manipulator == 'l_arm'):
			if(m1[0] != -99.0 and m2[0] != -99.0):
				desired_com = [(current_state.left_leg[0]+2.0*current_state.right_leg[0])/3.0, (current_state.left_leg[1]+2.0*current_state.right_leg[1])/3.0, (current_state.left_leg[2]+2.0*current_state.right_leg[2])/3.0 + robot_z]
			support_state = [True,True,False,ra[0]!=-99.0]
		elif(moving_manipulator == 'r_arm'):
			if(m1[0] != -99.0 and m2[0] != -99.0):
				desired_com = [(2.0*current_state.left_leg[0]+current_state.right_leg[0])/3.0, (2.0*current_state.left_leg[1]+current_state.right_leg[1])/3.0, (2.0*current_state.left_leg[2]+current_state.right_leg[2])/3.0 + robot_z]
			support_state = [True,True,la[0]!=-99.0,False]

		robot_center = desired_com

		mean_leg_transform = xyzrpy_to_SE3([(ll[0]+rl[0])/2.0, (ll[1]+rl[1])/2.0, (ll[2]+rl[2])/2.0, 0, 0, (ll[5]+rl[5])/2.0])

		if(m1[0] != -99.0 and m2[0] != -99.0): # switch to next contact position.
			transition_length = math.sqrt((m2[0]-m1[0])**2+(m2[1]-m1[1])**2+(m2[2]-m1[2])**2)
			transition_length_xy = math.sqrt((m2[0]-m1[0])**2+(m2[1]-m1[1])**2)
			transition_frame = np.array([[0,0,0,m1[0]],
										 [0,0,0,m1[1]],
										 [0,0,0,m1[2]],
										 [0,0,0,1]],float)

			nx = np.array([[0],[0],[0]],dtype=float)
			nx[0,0] = (m2[0]-m1[0])/transition_length
			nx[1,0] = (m2[1]-m1[1])/transition_length
			nx[2,0] = (m2[2]-m1[2])/transition_length

			nz = np.array([[0],[0],[0]],dtype=float)
			nz[0,0] = -(m2[1]-m1[1])/transition_length_xy
			nz[1,0] = (m2[0]-m1[0])/transition_length_xy
			nz[2,0] = 0

			if(nz[0,0]*(robot_center[0]-m1[0]) + nz[1,0]*(robot_center[1]-m1[1]) < 0):
				nz[0,0] = -nz[0,0]
				nz[1,0] = -nz[1,0]

			ny = np.array([[0],[0],[0]],dtype=float)
			ny[0,0] = nz[1,0] * nx[2,0] - nz[1,0] * nx[2,0]
			ny[1,0] = nz[2,0] * nx[0,0] - nz[2,0] * nx[0,0]
			ny[2,0] = nz[0,0] * nx[1,0] - nz[0,0] * nx[1,0]

			transition_frame[0:3,0:1] = nx
			transition_frame[0:3,1:2] = ny
			transition_frame[0:3,2:3] = nz

			inverse_transition_frame = inverse_SE3(transition_frame)

			m1_transform = xyzrpy_to_SE3(m1)
			m2_transform = xyzrpy_to_SE3(m2)
			m1_rotation = m1_transform[0:3,0:3]
			m2_rotation = m2_transform[0:3,0:3]

			(xy0,a,b) = hand_parabola_traj_fitting([0.0,0.0,0.0,0.0,0.0,0.0],[transition_length,0.0,0.0,0.0,0.0,0.0])
			step_dist = transition_length
		elif(m1[0] == -99.0 or m2[0] == -99.0):
			with robot:
				dof_values = robot.GetDOFValues()
				dof_values[DOFNameIndexDict['l_shoulder_pitch']] = 0.244
				dof_values[DOFNameIndexDict['l_shoulder_roll']] = 0.008
				dof_values[DOFNameIndexDict['l_shoulder_yaw']] = 0.089
				dof_values[DOFNameIndexDict['l_elbow_pitch']] = -0.523
				dof_values[DOFNameIndexDict['l_elbow_roll']] = 1.987
				dof_values[DOFNameIndexDict['l_wrist_pitch']] = 0.488
				dof_values[DOFNameIndexDict['l_wrist_yaw']] = 0.785
				dof_values[DOFNameIndexDict['r_shoulder_pitch']] = 0.244
				dof_values[DOFNameIndexDict['r_shoulder_roll']] = -0.008
				dof_values[DOFNameIndexDict['r_shoulder_yaw']] = -0.089
				dof_values[DOFNameIndexDict['r_elbow_pitch']] = -0.523
				dof_values[DOFNameIndexDict['r_elbow_roll']] = -1.987
				dof_values[DOFNameIndexDict['r_wrist_pitch']] = 0.488
				dof_values[DOFNameIndexDict['r_wrist_yaw']] = -0.785
				robot.SetDOFValues(dof_values)
				left_intermediate_transform = robot.GetManipulator('l_arm').GetTransform()
				right_intermediate_transform = robot.GetManipulator('r_arm').GetTransform()

			if(m1[0] != -99.0 and m2[0] == -99.0): # pull hand back.
				# straight line trajectory
				if(moving_manipulator == 'l_arm'):
					with robot:
						dof_values = robot.GetDOFValues()
						dof_values[DOFNameIndexDict['l_shoulder_pitch']] = 0.535
						dof_values[DOFNameIndexDict['l_shoulder_roll']] = 0.198
						dof_values[DOFNameIndexDict['l_shoulder_yaw']] = 0.0
						dof_values[DOFNameIndexDict['l_elbow_pitch']] = 0.503
						dof_values[DOFNameIndexDict['l_elbow_roll']] = 0.0
						dof_values[DOFNameIndexDict['l_wrist_pitch']] = 0.0
						dof_values[DOFNameIndexDict['l_wrist_yaw']] = 0.0
						robot.SetDOFValues(dof_values)
						original_left_hand_transform = robot.GetManipulator('l_arm').GetTransform()

					m1_transform = xyzrpy_to_SE3(m1)
					mint_transform = left_intermediate_transform
					# m2_transform = np.dot(mean_leg_transform,original_left_hand_transform)
					m2_transform = original_left_hand_transform
					m1_rotation = m1_transform[0:3,0:3]
					mint_rotation = mint_transform[0:3,0:3]
					m2_rotation = m2_transform[0:3,0:3]
				elif(moving_manipulator == 'r_arm'):
					with robot:
						dof_values = robot.GetDOFValues()
						dof_values[DOFNameIndexDict['r_shoulder_pitch']] = 0.535
						dof_values[DOFNameIndexDict['r_shoulder_roll']] = -0.198
						dof_values[DOFNameIndexDict['r_shoulder_yaw']] = 0.0
						dof_values[DOFNameIndexDict['r_elbow_pitch']] = 0.503
						dof_values[DOFNameIndexDict['r_elbow_roll']] = 0.0
						dof_values[DOFNameIndexDict['r_wrist_pitch']] = 0.0
						dof_values[DOFNameIndexDict['r_wrist_yaw']] = 0.0
						robot.SetDOFValues(dof_values)
						original_right_hand_transform = robot.GetManipulator('r_arm').GetTransform()

					m1_transform = xyzrpy_to_SE3(m1)
					mint_transform = right_intermediate_transform
					# m2_transform = np.dot(mean_leg_transform,original_right_hand_transform)
					m2_transform = original_right_hand_transform
					m1_rotation = m1_transform[0:3,0:3]
					mint_rotation = mint_transform[0:3,0:3]
					m2_rotation = m2_transform[0:3,0:3]

				m1_verti_transform = copy.copy(m1_transform)
				m1_verti_transform[:,3] = m1_transform[:,3]-init_vertical_translation*m1_transform[:,0]
				segment1_dist = init_vertical_translation
				segment2_dist = np.linalg.norm(mint_transform[:,3] - m1_verti_transform[:,3])
				segment3_dist = np.linalg.norm(m2_transform[:,3] - mint_transform[:,3])
				step_dist = segment1_dist + segment2_dist + segment3_dist
				subgoals = [(m1_transform,0),(m1_verti_transform,segment1_dist),(mint_transform,segment1_dist+segment2_dist),(m2_transform,segment1_dist+segment2_dist+segment3_dist)]

			elif(m1[0] == -99.0 and m2[0] != -99.0): # stretch hand out.
				# get the pose of current hand
				if(moving_manipulator == 'l_arm'):
					m1_transform = robot.GetManipulator('l_arm').GetTransform()
					mint_transform = left_intermediate_transform
					m2_transform = xyzrpy_to_SE3(m2)
					m1_rotation = m1_transform[0:3,0:3]
					mint_rotation = mint_transform[0:3,0:3]
					m2_rotation = m2_transform[0:3,0:3]
				elif(moving_manipulator == 'r_arm'):
					m1_transform = robot.GetManipulator('r_arm').GetTransform()
					mint_transform = right_intermediate_transform
					m2_transform = xyzrpy_to_SE3(m2)
					m1_rotation = m1_transform[0:3,0:3]
					mint_rotation = mint_transform[0:3,0:3]
					m2_rotation = m2_transform[0:3,0:3]

				m2_verti_transform = copy.copy(m2_transform)
				m2_verti_transform[:,3] = m2_transform[:,3]-init_vertical_translation*m2_transform[:,0]
				segment1_dist = np.linalg.norm(m1_transform[:,3] - mint_transform[:,3])
				segment2_dist = np.linalg.norm(mint_transform[:,3] - m2_verti_transform[:,3])
				segment3_dist = init_vertical_translation
				step_dist = segment1_dist + segment2_dist + segment3_dist
				subgoals = [(m1_transform,0),(mint_transform,segment1_dist),(m2_verti_transform,segment1_dist+segment2_dist),(m2_transform,segment1_dist+segment2_dist+segment3_dist)]

			subgoal_index = 1
		else:
			print('No way. BUG!!!!!')
			raw_input()



	# first shift com
	shift_com_result = shift_com(general_ik_interface,or_robot,com_start,desired_com,moving_manipulator,current_state,interpolate_step,traj,plan_waypoint_contact_manips,prev_contact_manip_group,manip_group_num)

	if shift_com_result is None:
		return None

	(prev_contact_manip_group,manip_group_num) = shift_com_result
	remain_dist = step_dist

	left_leg_tf = xyzrpy_to_SE3(ll)
	right_leg_tf = xyzrpy_to_SE3(rl)
	left_arm_tf = xyzrpy_to_SE3(la)
	right_arm_tf = xyzrpy_to_SE3(ra)

	reusegiwc = False

	# for loop to go through the interpolated point
	last_foot_xy_escape_dist = 0
	last_foot_z_escape_dist = 0
	last_hand_xy_escape_dist = 0
	while(remain_dist >= -0.001):
		# specify the support manipulator
		manip_poses = [left_leg_tf,right_leg_tf,left_arm_tf,right_arm_tf]

		polygon_trans = [0,0,0]

		traverse_dist = step_dist - remain_dist

		foot_xy_escape_dist = 0.6 * last_foot_xy_escape_dist
		foot_z_escape_dist = 0.6 * last_foot_z_escape_dist
		hand_xy_escape_dist = 0.6 * last_hand_xy_escape_dist

		moving_manip_in_collision = True
		while moving_manip_in_collision:

			if(moving_manipulator == 'l_leg' or moving_manipulator == 'r_leg'):
				if(traverse_dist >= 0 and traverse_dist < first_z_translation):
					mid_point = [m1[0],m1[1],m1[2]+(traverse_dist/first_z_translation)*first_z_translation,0,0,m1[5]]

				elif(traverse_dist >= first_z_translation and traverse_dist < first_z_translation+xy_translation):

					mid_point = [m1[0]+(m2[0]-m1[0])*(traverse_dist-first_z_translation)/(step_dist-first_z_translation),
								 m1[1]+(m2[1]-m1[1])*(traverse_dist-first_z_translation)/(step_dist-first_z_translation),
								 a*((traverse_dist-first_z_translation)-xy0)**2+b,
								 0, 0, (m1[5]*remain_dist + m2[5]*(traverse_dist-first_z_translation))/(step_dist-first_z_translation)]

					if foot_xy_escape_vector is not None:
						mid_point[0] += foot_xy_escape_dist*foot_xy_escape_vector[0]
						mid_point[1] += foot_xy_escape_dist*foot_xy_escape_vector[1]

					mid_point[2] += foot_z_escape_dist

				elif(traverse_dist >= first_z_translation+xy_translation and traverse_dist < first_z_translation+xy_translation+second_z_translation):
					mid_point = [m2[0],m2[1],m2[2]+(remain_dist/second_z_translation)*second_z_translation,m2[3],m2[4],m2[5]]
				else:
					mid_point = m2

				if(moving_manipulator == 'l_leg'):
					moving_left_leg_tf = xyzrpy_to_SE3(mid_point)
					manip_poses[0] = moving_left_leg_tf

				elif(moving_manipulator == 'r_leg'):
					moving_right_leg_tf = xyzrpy_to_SE3(mid_point)
					manip_poses[1] = moving_right_leg_tf

			elif(moving_manipulator == 'l_arm' or moving_manipulator == 'r_arm'):
				# parabola trajectory
				if(m1[0] != -99.0 and m2[0] != -99.0):
					traverse_ratio = traverse_dist / step_dist
					mid_point_position = np.array([[traverse_dist],[0],[a*(traverse_dist-xy0)**2+b+hand_xy_escape_dist],[1]],dtype=float)
					mid_point_position = np.dot(transition_frame,mid_point_position)
				else:
					for index,subgoal in enumerate(subgoals):
						if(subgoal[1] > traverse_dist):
							subgoal_index = min(len(subgoals)-1,index)
							traverse_ratio = (traverse_dist - subgoals[index-1][1]) / (subgoals[index][1]-subgoals[index-1][1])
							break

					mid_point_position = (1-traverse_ratio) * subgoals[index-1][0][:,3:4] + traverse_ratio * subgoals[index][0][:,3:4]
					m1_rotation = subgoals[index-1][0][0:3,0:3]
					m2_rotation = subgoals[index][0][0:3,0:3]

				if(moving_manipulator == 'l_arm'):
					moving_left_arm_tf = np.zeros((4,4),dtype=float)
					moving_left_arm_tf[0:3,0:3] = rotation_interpolation(m1_rotation,m2_rotation,min(traverse_ratio*1.2,1.0))
					moving_left_arm_tf[0:4,3:4] = mid_point_position
					manip_poses[2] = moving_left_arm_tf
				elif(moving_manipulator == 'r_arm'):
					moving_right_arm_tf = np.zeros((4,4),dtype=float)
					moving_right_arm_tf[0:3,0:3] = rotation_interpolation(m1_rotation,m2_rotation,min(traverse_ratio*1.2,1.0))
					moving_right_arm_tf[0:4,3:4] = mid_point_position
					manip_poses[3] = moving_right_arm_tf

			robot.SetActiveDOFs(active_dof_indices)

			desired_com = np.array([0,0,0],dtype=float)
			weight = 0.0
			if support_state[0]:
				desired_com += left_leg_tf[0:3,3]
				weight += 1.0

			if support_state[1]:
				desired_com += right_leg_tf[0:3,3]
				weight += 1.0

			desired_com = (1.0/weight) * desired_com
			desired_com[2] += 0.9

			solved_joint_values = IK(general_ik_interface,robot,manip_poses,support_state,desired_com,return_closest_solution=False,reusegiwc=reusegiwc)

			if solved_joint_values is None:
				solved_joint_values = IK(general_ik_interface,robot,manip_poses,support_state,desired_com,return_closest_solution=True,reusegiwc=reusegiwc)

			reusegiwc = True


			robot.SetActiveDOFs(whole_body_indices)


			if(solved_joint_values is None):
				print('IK fail in transitioning manipulators.')
				return None
			else:
				solved_joint_values = robot.GetActiveDOFValues()
				for d in range(len(solved_joint_values)):
					if(solved_joint_values[d] <= lower_limits[d] + 0.000001):
						solved_joint_values[d] = lower_limits[d] + 0.0001
					elif(solved_joint_values[d] >= higher_limits[d] - 0.000001):
						solved_joint_values[d] = higher_limits[d] - 0.0001

				robot.SetActiveDOFValues(solved_joint_values)

				# check for self collision
				moving_foot_in_self_collision = False
				moving_foot_in_collision = False
				moving_hand_in_collision = False

				if ((moving_manipulator == 'l_leg' or moving_manipulator == 'r_leg') and
					 (traverse_dist >= first_z_translation and traverse_dist < first_z_translation+xy_translation)):

					# foot xy escape
					if foot_xy_escape_vector is not None:
						if moving_manipulator == 'l_leg':
							moving_foot_in_self_collision = env.CheckCollision(robot.GetLink('l_foot'),robot.GetLink('r_shin'))
						elif moving_manipulator == 'r_leg':
							moving_foot_in_self_collision = env.CheckCollision(robot.GetLink('r_foot'),robot.GetLink('l_shin'))

					# foot z escape
					if not moving_foot_in_self_collision:
						if moving_manipulator == 'l_leg':
							moving_foot_in_collision = env.CheckCollision(robot.GetLink('l_foot'))
						elif moving_manipulator == 'r_leg':
							moving_foot_in_collision = env.CheckCollision(robot.GetLink('r_foot'))

				if ((moving_manipulator == 'l_arm' or moving_manipulator == 'r_arm') and
					(m1[0] != -99.0 and m2[0] != -99.0)):
					# hand xy escape
					if moving_manipulator == 'l_arm':
						moving_hand_in_collision = env.CheckCollision(robot.GetLink('l_palm'))
					elif moving_manipulator == 'r_arm':
						moving_hand_in_collision = env.CheckCollision(robot.GetLink('r_palm'))

				moving_manip_in_collision = moving_foot_in_self_collision or moving_foot_in_collision or moving_hand_in_collision

				if moving_foot_in_self_collision:
					foot_xy_escape_dist += 0.005

				if moving_foot_in_collision:
					foot_z_escape_dist += 0.005

				if moving_hand_in_collision:
					hand_xy_escape_dist += 0.005

		last_foot_xy_escape_dist = foot_xy_escape_dist
		last_foot_z_escape_dist = foot_z_escape_dist
		last_hand_xy_escape_dist = hand_xy_escape_dist

		traj.Insert(traj.GetNumWaypoints(),solved_joint_values)

		current_contact_manips = []
		for i,support in enumerate(support_state):
			if(support):
				support_manip = support_state_mapping.get(i)
				if(prev_contact_manip_group.get(support_manip) is not None):
					current_contact_manips.append((support_manip,prev_contact_manip_group.get(support_manip)))
				else:
					current_contact_manips.append((support_manip,manip_group_num))
					manip_group_num = manip_group_num + 1

		prev_contact_manip_group = {}

		for manip in current_contact_manips:
			prev_contact_manip_group[manip[0]] = manip[1]

		plan_waypoint_contact_manips.append(current_contact_manips)


		if(remain_dist > 0.0 and remain_dist < interpolate_step):
			remain_dist = 0.0
		else:
			remain_dist = remain_dist - interpolate_step


	return (prev_contact_manip_group,manip_group_num)



def shift_com(general_ik_interface,or_robot,com_start,com_goal,moving_manipulator,current_state,interpolate_step,traj,plan_waypoint_contact_manips,prev_contact_manip_group,manip_group_num):
	# take straight line com path
	com_dist = math.sqrt(pow(com_goal[0]-com_start[0],2) + pow(com_goal[1]-com_start[1],2))
	remain_dist = com_dist

	ll = current_state.left_leg
	rl = current_state.right_leg
	la = current_state.left_arm
	ra = current_state.right_arm

	robot_z = or_robot.robot_z
	robot = or_robot.robot

	l_arm_indices = robot.GetManipulator('l_arm').GetArmIndices()
	r_arm_indices = robot.GetManipulator('r_arm').GetArmIndices()
	l_leg_indices = robot.GetManipulator('l_leg').GetArmIndices()
	r_leg_indices = robot.GetManipulator('r_leg').GetArmIndices()

	additional_active_DOF_indices = [None]*len(or_robot.additional_active_DOFs)
	for index,j in enumerate(or_robot.additional_active_DOFs):
		additional_active_DOF_indices[index] = robot.GetJoint(j).GetDOFIndex()

	whole_body_indices = np.concatenate((l_arm_indices, r_arm_indices, l_leg_indices, r_leg_indices, additional_active_DOF_indices))

	manip_state = [ll,rl,la,ra]
	contact_state = [True,True,la[0]!=-99.0,ra[0]!=-99.0]
	support_state = [True,True,la[0]!=-99.0,ra[0]!=-99.0]

	# Force the robot to shift its CoM to one of the leg
	if(moving_manipulator == 'l_leg'):
		support_state = [False,True,la[0]!=-99.0,ra[0]!=-99.0]
	elif(moving_manipulator == 'r_leg'):
		support_state = [True,False,la[0]!=-99.0,ra[0]!=-99.0]

	manip_index_mapping = {0:'l_leg',1:'r_leg',2:'l_arm',3:'r_arm'}

	if(la[0] == -99.0 and ra[0] == -99.0):
		active_dof_indices = np.concatenate((l_leg_indices, r_leg_indices, additional_active_DOF_indices))
	elif(la[0] != -99.0 and ra[0] == -99.0):
		active_dof_indices = np.concatenate((l_arm_indices, l_leg_indices, r_leg_indices, additional_active_DOF_indices))
	elif(la[0] == -99.0 and ra[0] != -99.0):
		active_dof_indices = np.concatenate((r_arm_indices, l_leg_indices, r_leg_indices, additional_active_DOF_indices))
	else:
		active_dof_indices = whole_body_indices

	reusegiwc = False

	while(remain_dist >= 0):
		desired_com = [(1-remain_dist/com_dist)*(com_goal[0]-com_start[0]) + com_start[0], (1-remain_dist/com_dist)*(com_goal[1]-com_start[1]) + com_start[1], robot_z + (ll[2]+rl[2])/2.0]

		robot.SetActiveDOFs(active_dof_indices)

		solved_joint_values = IK(general_ik_interface,robot,manip_state,support_state,desired_com,reusegiwc=reusegiwc,return_closest_solution=True,exact_com=True)
		reusegiwc = True

		# Note that we allow IK to return the closest solution in this case, it is impossible to be invoked.
		if solved_joint_values is None:
			print('IK fail when shifting CoM.')
			return None

		robot.SetActiveDOFs(whole_body_indices)

		solved_joint_values = robot.GetActiveDOFValues()

		traj.Insert(traj.GetNumWaypoints(),solved_joint_values)

		current_contact_manips = []
		for i,in_contact in enumerate(contact_state):
			if(in_contact):
				contact_manip = manip_index_mapping.get(i)
				if(prev_contact_manip_group.get(contact_manip) is not None):
					current_contact_manips.append((contact_manip,prev_contact_manip_group.get(contact_manip)))
				else:
					current_contact_manips.append((contact_manip,manip_group_num))
					manip_group_num = manip_group_num + 1

		prev_contact_manip_group = {}

		for manip in current_contact_manips:
			prev_contact_manip_group[manip[0]] = manip[1]

		plan_waypoint_contact_manips.append(current_contact_manips)

		remain_dist = remain_dist - interpolate_step
		if(remain_dist > -interpolate_step):
			remain_dist = max(remain_dist,0.0)

	return (prev_contact_manip_group,manip_group_num)


def IK(general_ik_interface,robot,manip_poses,support_state,desired_com=None,reusegiwc=False,or_robot=None,return_closest_solution=False,exact_com=False):

	# balance_checker = 'support_polygon'
	balance_checker = 'giwc'

	maniptm_list = []
	support_list = []
	support_links = []

	ll = manip_poses[0]; ll_support = support_state[0]
	rl = manip_poses[1]; rl_support = support_state[1]
	la = manip_poses[2]; la_support = support_state[2]
	ra = manip_poses[3]; ra_support = support_state[3]

	# left leg
	if(isinstance(ll, list)):
		left_leg_tf = xyzrpy_to_SE3(ll)
	elif(isinstance(ll, np.ndarray)):
		left_leg_tf = copy.deepcopy(ll)
	maniptm_list.append(('l_leg',left_leg_tf))
	if(ll_support):
		support_list.append(('l_leg',mu))
		support_links.append('l_foot')

	# right leg
	if(isinstance(rl, list)):
		right_leg_tf = xyzrpy_to_SE3(rl)
	elif(isinstance(rl, np.ndarray)):
		right_leg_tf = copy.deepcopy(rl)
	maniptm_list.append(('r_leg',right_leg_tf))
	if(rl_support):
		support_list.append(('r_leg',mu))
		support_links.append('r_foot')

	# left arm
	if((isinstance(la, list) and la[0] != -99.0) or (isinstance(la, np.ndarray) and la[0,3] != -99.0)):
		if(isinstance(la, list)):
			left_arm_tf = xyzrpy_to_SE3(la)
		elif(isinstance(la, np.ndarray)):
			left_arm_tf = copy.deepcopy(la)
		maniptm_list.append(('l_arm',left_arm_tf))
		if(la_support):
			support_list.append(('l_arm',mu))
			balance_checker = 'giwc'

	# right arm
	if((isinstance(ra, list) and ra[0] != -99.0) or (isinstance(ra, np.ndarray) and ra[0,3] != -99.0)):
		if(isinstance(ra, list)):
			right_arm_tf = xyzrpy_to_SE3(ra)
		elif(isinstance(ra, np.ndarray)):
			right_arm_tf = copy.deepcopy(ra)
		maniptm_list.append(('r_arm',right_arm_tf))
		if(ra_support):
			support_list.append(('r_arm',mu))
			balance_checker = 'giwc'

	(tmp_lower_limits,tmp_higher_limits) = robot.GetActiveDOFLimits()

	# IK
	if(exact_com):
		if desired_com is None:
			print('Command to use exact_com, but the com is not specified.')
			return None

		solved_joint_values = general_ik_interface.DoGeneralIK(execute=True, returnclosest=return_closest_solution, maniptm=maniptm_list, movecog=desired_com, exact_com=exact_com, printcommand=False)
		# solved_joint_values = general_ik_interface.DoGeneralIK(execute=True, returnclosest=return_closest_solution, maniptm=maniptm_list, support=support_list, movecog=desired_com, exact_com=exact_com, printcommand=False)
	else:
		solved_joint_values = general_ik_interface.DoGeneralIK(execute=True, returnclosest=return_closest_solution, maniptm=maniptm_list, printcommand=False)

		ActiveDOFValues = robot.GetActiveDOFValues()
		for d in range(len(ActiveDOFValues)):
			if(ActiveDOFValues[d] <= tmp_lower_limits[d] + 0.000001):
				ActiveDOFValues[d] = tmp_lower_limits[d] + 0.001
			elif(ActiveDOFValues[d] >= tmp_higher_limits[d] - 0.000001):
				ActiveDOFValues[d] = tmp_higher_limits[d] - 0.001
		robot.SetActiveDOFValues(ActiveDOFValues)


		if(balance_checker == 'giwc'):
			if desired_com is None:
				solved_joint_values = general_ik_interface.DoGeneralIK(execute=True, returnclosest=return_closest_solution, maniptm=maniptm_list, support=support_list, reusegiwc=reusegiwc, printcommand=False)
			else:
				solved_joint_values = general_ik_interface.DoGeneralIK(execute=True, returnclosest=return_closest_solution, maniptm=maniptm_list, support=support_list, movecog=desired_com, reusegiwc=reusegiwc, printcommand=False)


		elif(balance_checker == 'support_polygon'):
			if desired_com is None:
				solved_joint_values = general_ik_interface.DoGeneralIK(execute=True, returnclosest=return_closest_solution, maniptm=maniptm_list, supportlinks=support_links, polyscale=poly_scale, printcommand=False)
			else:
				solved_joint_values = general_ik_interface.DoGeneralIK(execute=True, returnclosest=return_closest_solution, maniptm=maniptm_list, movecog=desired_com, supportlinks=support_links, polyscale=poly_scale, printcommand=False)


	if solved_joint_values is not None:
		for d in range(len(solved_joint_values)):
			if(solved_joint_values[d] <= tmp_lower_limits[d] + 0.000001):
				solved_joint_values[d] = tmp_lower_limits[d] + 0.001
			elif(solved_joint_values[d] >= tmp_higher_limits[d] - 0.000001):
				solved_joint_values[d] = tmp_higher_limits[d] - 0.001


	return solved_joint_values


def step_interpolation(rave_traj,path,or_robot,reset_robot_config=True,general_ik_interface=None):
	print('step interpolation')

	DOFNameIndexDict = or_robot.DOFNameIndexDict
	DOFNameActiveIndexDict = or_robot.DOFNameActiveIndexDict
	lower_limits = or_robot.lower_limits
	higher_limits = or_robot.higher_limits
	IKInitDOFValues = or_robot.IKInitDOFValues
	StandingPostureDOFValues = or_robot.StandingPostureDOFValues
	robot_z = or_robot.robot_z
	robot = or_robot.robot

	if general_ik_interface is None:
		general_ik_interface = CBiRRT(or_robot.env, or_robot.robot.GetName())

	plan_waypoint_contact_manips = []
	prev_contact_manip_group = {'l_leg':0,'r_leg':1} # (manip name,contact group)
	manip_group_num = 2

	traj_feasible = True
	interpolate_step = 0.01

	if(len(path) != 0):

		# bug detection
		for i, state in enumerate(path):
			if(i != 0):
				# sheck if parent equals the previous index state
				if(path[i].parent != path[i-1]):
					print('ERROR: State parent and previous state in the path mismatch.')
					IPython.embed()

				# check if there is any difference between consecutive states
				diff_manip = []
				if(not np.array_equal(np.array(state.left_leg),np.array(state.parent.left_leg))):
					diff_manip.append(0)

				if(not np.array_equal(np.array(state.right_leg),np.array(state.parent.right_leg))):
					diff_manip.append(1)

				if(not np.array_equal(np.array(state.left_arm),np.array(state.parent.left_arm))):
					diff_manip.append(2)

				if(not np.array_equal(np.array(state.right_arm),np.array(state.parent.right_arm))):
					diff_manip.append(3)

				if(len(diff_manip) != 1):
					print('ERROR: Multiple or no moving manipulator.')
					IPython.embed()

				# check if the prev_move_manip matches the motion
				if(state.prev_move_manip not in diff_manip):
					print('ERROR: prev_move_manip differs from actual manipulator movements.')
					IPython.embed()

		if reset_robot_config:
			robot.SetDOFValues(StandingPostureDOFValues)
			first_dof_values = robot.GetDOFValues()
			first_dof_values[DOFNameIndexDict['x_prismatic_joint']] = (path[0].left_leg[0]+path[0].right_leg[0])/2.0
			first_dof_values[DOFNameIndexDict['y_prismatic_joint']] = (path[0].left_leg[1]+path[0].right_leg[1])/2.0
			first_dof_values[DOFNameIndexDict['z_prismatic_joint']] = (path[0].left_leg[2]+path[0].right_leg[2])/2.0 + robot_z
			first_dof_values[DOFNameIndexDict['yaw_revolute_joint']] = path[0].get_virtual_body_yaw() * deg_to_rad
			first_dof_values[DOFNameIndexDict['l_wrist_yaw']] = 0.1
			first_dof_values[DOFNameIndexDict['r_wrist_yaw']] = -0.1
			robot.SetDOFValues(first_dof_values)

		# get the first dof
		first_waypoint_contact_manips = [('l_leg',0),('r_leg',1)]
		if(path[0].left_arm[0] != -99.0):
			first_waypoint_contact_manips.append(('l_arm',len(first_waypoint_contact_manips)))

		if(path[0].right_arm[0] != -99.0):
			first_waypoint_contact_manips.append(('r_arm',len(first_waypoint_contact_manips)))

		plan_waypoint_contact_manips.append(first_waypoint_contact_manips)
		first_manip_poses = [path[0].left_leg,path[0].right_leg,path[0].left_arm,path[0].right_arm]
		support_state = [True,True,path[0].left_arm[0]!=-99.0,path[0].right_arm[0]!=-99.0]
		desired_com = [(path[0].left_leg[0]+path[0].right_leg[0])/2.0, (path[0].left_leg[1]+path[0].right_leg[1])/2.0, (path[0].left_leg[2]+path[0].right_leg[2])/2.0 + robot_z]

		solved_joint_values = IK(general_ik_interface,robot,first_manip_poses,support_state,desired_com,return_closest_solution=True)
		if solved_joint_values is None:
			print('IK fail in the initial waypoint.')
			return (False,None)

		rave_traj.Insert(rave_traj.GetNumWaypoints(),solved_joint_values)

		moving_manip_log = []


		for i in range(len(path)):
			# if IK in one of the waypoint fails, break.
			if(not traj_feasible):
				break

			if(i != len(path)-1):
				s1 = path[i]
				s2 = path[i+1]

				ll1 = s1.left_leg
				rl1 = s1.right_leg
				la1 = s1.left_arm
				ra1 = s1.right_arm

				ll2 = s2.left_leg
				rl2 = s2.right_leg
				la2 = s2.left_arm
				ra2 = s2.right_arm

				if(s2.prev_move_manip == 0):
					m1 = ll1; m2 = ll2; moving_manipulator = 'l_leg'
				elif(s2.prev_move_manip == 1):
					m1 = rl1; m2 = rl2; moving_manipulator = 'r_leg'
				elif(s2.prev_move_manip == 2):
					m1 = la1; m2 = la2; moving_manipulator = 'l_arm'
				elif(s2.prev_move_manip == 3):
					m1 = ra1; m2 = ra2; moving_manipulator = 'r_arm'
				else:
					print('BUG!!!!!')
					IPython.embed()

				moving_manip_log.append(moving_manipulator)

				transition_manipulator_result = transition_manipulator(general_ik_interface,or_robot,path[i],m1,m2,moving_manipulator,interpolate_step,rave_traj,plan_waypoint_contact_manips,prev_contact_manip_group,manip_group_num)

				if transition_manipulator_result is None:
					return (False,None)

				(prev_contact_manip_group,manip_group_num) = transition_manipulator_result
				# raw_input()

		# insert one last waypoint to finish the final contact
		last_state = path[len(path)-1]
		manip_poses = [last_state.left_leg,last_state.right_leg,last_state.left_arm,last_state.right_arm]
		support_state = [True,True,last_state.left_arm[0]!=-99.0,last_state.right_arm[0]!=-99.0]
		desired_com = [(last_state.left_leg[0]+last_state.right_leg[0])/2.0, (last_state.left_leg[1]+last_state.right_leg[1])/2.0, (last_state.left_leg[2]+last_state.right_leg[2])/2.0 + robot_z]
		if(len(path) != 1):
			solved_joint_values = IK(general_ik_interface,robot,manip_poses,support_state,desired_com,return_closest_solution=True)
			if solved_joint_values is None:
				print('IK fail in the last waypoint.')
				return (False,None)
			rave_traj.Insert(rave_traj.GetNumWaypoints(),solved_joint_values)
		else:
			rave_traj.Insert(0,robot.GetActiveDOFValues())

		current_contact_manips = []
		support_state_mapping = {0:'l_leg',1:'r_leg',2:'l_arm',3:'r_arm'}
		for i,support in enumerate(support_state):
			if(support):
				support_manip = support_state_mapping.get(i)
				if(prev_contact_manip_group.get(support_manip) is not None):
					current_contact_manips.append((support_manip,prev_contact_manip_group.get(support_manip)))
				else:
					current_contact_manips.append((support_manip,manip_group_num))
					manip_group_num = manip_group_num + 1

		plan_waypoint_contact_manips.append(current_contact_manips)


	# planning fails, no solution.
	else:
		traj_feasible = False
		raw_input('No solution.')

	return (traj_feasible,plan_waypoint_contact_manips)