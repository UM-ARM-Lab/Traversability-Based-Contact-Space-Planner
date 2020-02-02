from robot import HumanoidRobot
from transformation_conversion import *

import numpy as np
import openravepy as rave
import math
import IPython


def escher(env, active_dof_mode='whole_body', urdf_name=None):

    # Specify the urdf and srdf path
    urdf = 'package://escher_description/robot/escher.urdf'
    srdf = 'package://escher_description/robot/escher.srdf'

    if urdf_name is not None:
        urdf = 'package://escher_description/robot/' + urdf_name + '.urdf'

    escher = HumanoidRobot(env, urdf_path=urdf, srdf_path=srdf)

    # Set up the end-effector frame for each manipulators
    escher.manip.l_arm.SetLocalToolDirection(np.array([1, 0, 0]))
    escher.manip.l_arm.SetLocalToolTransform(np.array([
        [0,  1, 0, 0.086],
        [ -1, 0, 0, -0.03],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1]])
    )
    escher.manip.r_arm.SetLocalToolDirection(np.array([1, 0, 0]))
    escher.manip.r_arm.SetLocalToolTransform(np.array([
        [ 0,  -1, 0, 0.086],
        [ 1,  0, 0, 0.03],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1]])
    )
    escher.manip.l_leg.SetLocalToolDirection(np.array([0, 0, -1]))
    escher.manip.r_leg.SetLocalToolDirection(np.array([0, 0, -1]))

    # Specify the end-effector dimensions. We assume the robot has rectangular contacts.
    escher.foot_h = 0.25
    escher.foot_w = 0.135
    escher.hand_h = 0.20
    escher.hand_w = 0.14
    escher.foot_radius = math.sqrt((escher.foot_h/2.0)**2 + (escher.foot_w/2.0)**2)
    escher.hand_radius = math.sqrt((escher.hand_h/2.0)**2 + (escher.hand_w/2.0)**2)

    # Specify the robot's dimension
    escher.robot_z = 0.9 # robot waist height
    escher.top_z = 1.85 # total robot height
    escher.shoulder_z = 1.45 # robot shoulder height
    escher.shoulder_w = 0.6 # robot shoulder to shoulder distance
    escher.max_arm_length = 0.7
    escher.min_arm_length = 0.3
    escher.max_arm_forward_angle = 180
    escher.max_arm_backward_angle = 180

    # Specify the robot's movement range
    escher.foot_transition_long_radius = 0.7 # the max distance a foot can travel with a forward stride.
    escher.foot_transition_short_radius = 0.3 # the max distance a foot can travel with a sideway stride.
    escher.max_body_transition_dist = 0.4 # the max distance the robot's body (mean position of the feet) can move for a contact transition. (An estimate)
    escher.max_hand_transition_dist = 0.6 # the max distance the robot's hand can move for a contact transition. (An estimate)

    # Specify the active DOFs
    escher.additional_active_DOFs = ['x_prismatic_joint','y_prismatic_joint','z_prismatic_joint','roll_revolute_joint','pitch_revolute_joint','yaw_revolute_joint','waist_yaw']
    l_arm_indices = escher.robot.GetManipulator('l_arm').GetArmIndices()
    r_arm_indices = escher.robot.GetManipulator('r_arm').GetArmIndices()
    l_leg_indices = escher.robot.GetManipulator('l_leg').GetArmIndices()
    r_leg_indices = escher.robot.GetManipulator('r_leg').GetArmIndices()
    additional_active_DOF_indices = [escher.robot.GetJoint(joint_index).GetDOFIndex() for joint_index in escher.additional_active_DOFs]
    whole_body_indices = np.concatenate((l_arm_indices, r_arm_indices, l_leg_indices, r_leg_indices, additional_active_DOF_indices))
    if active_dof_mode == 'whole_body':
        escher.robot.SetActiveDOFs(whole_body_indices)

    # initialize robot config data (DOFName-DOFIndex Dictionary, joint limits)
    escher.initialize_config_data()

    # Initialize robot end-effector bounding box used for collision check in contact space planning.
    escher.initialize_end_effector_collision_box()

    # Initialize robot body bounding box used for collision check in guiding path planning.
    out_of_env_transform = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-99.0],[0,0,0,1]])
    escher.body_collision_box = rave.RaveCreateKinBody(escher.env,'')
    escher.body_collision_box.SetName('body_collision_box')
    escher.body_collision_box.InitFromBoxes(np.array([[0,0,(0.5+escher.top_z)/2.0,escher.foot_h*0.95,escher.shoulder_w/2.0+0.2,(escher.top_z-0.5)/2.0]]),False)
    escher.env.AddKinBody(escher.body_collision_box)
    escher.body_collision_box.SetTransform(out_of_env_transform)
    escher.body_collision_box_offset = np.array([[1,0,0,0],
                                                 [0,1,0,0],
                                                 [0,0,1,0],
                                                 [0,0,0,1]], dtype=float)
    escher.inverse_body_collision_box_offset = inverse_SE3(escher.body_collision_box_offset)
    escher.origin_body_transform = np.array([[1,0,0,0],
                                             [0,1,0,0],
                                             [0,0,1,0],
                                             [0,0,0,1]], dtype=float)

    # Construct the StandingPostureDOFValues, the DOF Values that the robot stands with arms laying down.
    # It is used as the initial configuration of the trajectory of a planned contact sequence.
    escher.StandingPostureDOFValues = np.array([ 0.00000000e+00,   1.30580638e-02,  -1.40269059e-03,
                                               -3.86412875e-01,   7.32566641e-04,   5.02539889e-01,
                                                4.82786220e-05,  -3.05805816e-01,  -4.29312864e-04,
                                                2.13561552e-04,   5.81795021e-08,   6.68554145e-01,
                                                1.14303672e-07,   5.35135688e-01,   1.97788584e-01,
                                                1.86868359e-04,  -1.91525395e-07,  -8.35585732e-06,
                                                3.75628690e-05,   6.03302644e-05,   0.00000000e+00,
                                               -3.85086517e-01,   8.67205998e-04,   4.98963063e-01,
                                                7.35581810e-05,  -3.07858142e-01,  -5.49373110e-04,
                                               -9.83564221e-04,   6.83513540e-07,   6.70171382e-01,
                                                7.31314300e-07,   5.38380011e-01,  -1.98555985e-01,
                                                2.62093203e-03,   1.86053196e-06,   1.16882178e-05,
                                                1.12270384e-03,   1.49762135e-04,  -3.17439990e-04,
                                               -4.27539115e-07,   2.45785498e-02,   6.09822426e-04,
                                                9.91970073e-04,  -4.64415464e-02])

    # Construct the IKInitDOFValues, the DOF Value we set before calling IK Solver to find kinematic solutions.
    escher.robot.SetDOFValues(escher.StandingPostureDOFValues)

    DOFValues = escher.robot.GetDOFValues()
    DOFValues[escher.DOFNameIndexDict['r_shoulder_pitch']] = -math.pi/4
    DOFValues[escher.DOFNameIndexDict['r_shoulder_roll']] = -math.pi/4
    DOFValues[escher.DOFNameIndexDict['r_shoulder_yaw']] = -0.5
    DOFValues[escher.DOFNameIndexDict['r_elbow_pitch']] = math.pi/4
    DOFValues[escher.DOFNameIndexDict['r_elbow_roll']] = -math.pi/2
    DOFValues[escher.DOFNameIndexDict['r_wrist_pitch']] = 0
    DOFValues[escher.DOFNameIndexDict['r_wrist_yaw']] = -math.pi/4

    DOFValues[escher.DOFNameIndexDict['l_shoulder_pitch']] = -math.pi/4
    DOFValues[escher.DOFNameIndexDict['l_shoulder_roll']] = math.pi/4
    DOFValues[escher.DOFNameIndexDict['l_shoulder_yaw']] = 0.5
    DOFValues[escher.DOFNameIndexDict['l_elbow_pitch']] = math.pi/4
    DOFValues[escher.DOFNameIndexDict['l_elbow_roll']] = math.pi/2
    DOFValues[escher.DOFNameIndexDict['l_wrist_pitch']] = 0
    DOFValues[escher.DOFNameIndexDict['l_wrist_yaw']] = math.pi/4

    escher.robot.SetDOFValues(DOFValues)

    ActiveDOFValues = escher.robot.GetActiveDOFValues()
    for d in range(len(ActiveDOFValues)):
        if ActiveDOFValues[d] <= escher.lower_limits[d] + 0.000001:
            ActiveDOFValues[d] = escher.lower_limits[d] + 0.0001
        elif ActiveDOFValues[d] >= escher.higher_limits[d] - 0.000001:
            ActiveDOFValues[d] = escher.higher_limits[d] - 0.0001
    escher.robot.SetActiveDOFValues(ActiveDOFValues)

    escher.IKInitDOFValues = escher.robot.GetDOFValues()

    escher.mass = 50 # not used

    return escher