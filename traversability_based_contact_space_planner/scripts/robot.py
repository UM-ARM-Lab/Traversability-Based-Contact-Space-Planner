from __future__ import print_function

import openravepy as rave
import numpy as np

from transformation_conversion import *

class AttributePassthrough(object):
    def __init__(self, getter, getAll):
        self.getter = getter
        self.getAll = getAll

    def __getattr__(self, item):
        return self.getter(item)

    def __getitem__(self, item):
        return self.getter(item)

    def __iter__(self):
        return iter(self.getAll())

class HumanoidRobot:
    _waiting_for_pose = False
    _waiting_for_joints = False
    _disable_state_updates = False

    def __init__(self, env, urdf_path, srdf_path):
        self.env = env

        # Load robot
        module = rave.RaveCreateModule(self.env, 'urdf')
        robot_name = module.SendCommand('loadURI {} {}'.format(urdf_path, srdf_path))

        self.robot = self.env.GetRobot(robot_name)

        ###############################################################

        self.DOFNameIndexDict = {}
        self.DOFNameActiveIndexDict = {}
        self.lower_limits = []
        self.higher_limits = []
        self.IKInitDOFValues = []
        self.StandingPostureDOFValues = []
        self.foot_h = None
        self.foot_w = None
        self.hand_h = None
        self.hand_w = None
        self.foot_radius = None
        self.hand_radius = None

        self.robot_z = None
        self.top_z = None
        self.shoulder_z = None
        self.shoulder_w = None

        self.max_arm_length = None
        self.min_arm_length = None

        self.foot_transition_long_radius = None
        self.foot_transition_short_radius = None
        self.max_foot_transition_dist = None
        self.max_hand_transition_dist = None

        ###############################################################

        self.foot_collision_box_1 = None
        self.foot_collision_box_2 = None
        self.foot_collision_box_3 = None
        self.foot_collision_box_4 = None
        self.hand_collision_box_1 = None
        self.hand_collision_box_2 = None
        self.hand_collision_box_3 = None

        self.body_collision_box = None

        self.body_collision_box_offset = None
        self.inverse_body_collision_box_offset = None
        self.origin_body_transform = None

        ###############################################################

        self.link = AttributePassthrough(self.robot.GetLink, self.robot.GetLinks)
        self.joint = AttributePassthrough(self.robot.GetJoint, self.robot.GetJoints)
        self.manip = AttributePassthrough(self.robot.GetManipulator, self.robot.GetManipulators)

        self.react_goal = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return

    def initialize_config_data(self):
        for dof in self.robot.GetJoints():
            self.DOFNameIndexDict[dof.GetName()] = dof.GetJointIndex()

        for active_dof_index, index in enumerate(self.robot.GetActiveDOFIndices()):
            self.DOFNameActiveIndexDict[self.robot.GetJointFromDOFIndex(index).GetName()] = active_dof_index

        (self.lower_limits,self.higher_limits) = self.robot.GetActiveDOFLimits()

    def initialize_end_effector_collision_box(self):

        out_of_env_transform = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-99.0],[0,0,0,1]])

        self.foot_collision_box_1 = rave.RaveCreateKinBody(self.env,'')
        self.foot_collision_box_1.SetName('foot_collision_box_1')
        foot_collision_box_parameter = np.array([[0,0,0.105,self.foot_h/2.0,self.foot_w/2.0,0.1]])
        self.foot_collision_box_1.InitFromBoxes(foot_collision_box_parameter,False)
        self.env.AddKinBody(self.foot_collision_box_1)
        self.foot_collision_box_1.SetTransform(out_of_env_transform)

        self.foot_collision_box_2 = rave.RaveCreateKinBody(self.env,'')
        self.foot_collision_box_2.SetName('foot_collision_box_2')
        foot_collision_box_parameter = np.array([[0.05,0,0.3,self.foot_h/2.0+0.2,self.foot_w/2.0+0.1,0.105]])
        # foot_collision_box_parameter = np.array([[0,0,0.3,self.foot_h/2.0+0.1,self.foot_w/2.0+0.3,0.105]])
        self.foot_collision_box_2.InitFromBoxes(foot_collision_box_parameter,False)
        self.env.AddKinBody(self.foot_collision_box_2)
        self.foot_collision_box_2.SetTransform(out_of_env_transform)

        self.foot_collision_box_3 = rave.RaveCreateKinBody(self.env,'')
        self.foot_collision_box_3.SetName('foot_collision_box_3')
        foot_collision_box_parameter = np.array([[0,0,0.105,self.foot_h/2.0,self.foot_w/2.0,0.1]])
        self.foot_collision_box_3.InitFromBoxes(foot_collision_box_parameter,False)
        self.env.AddKinBody(self.foot_collision_box_3)
        self.foot_collision_box_3.SetTransform(out_of_env_transform)

        self.foot_collision_box_4 = rave.RaveCreateKinBody(self.env,'')
        self.foot_collision_box_4.SetName('foot_collision_box_4')
        foot_collision_box_parameter = np.array([[0.05,0,0.3,self.foot_h/2.0+0.2,self.foot_w/2.0+0.1,0.105]])
        # foot_collision_box_parameter = np.array([[0,0,0.3,self.foot_h/2.0+0.1,self.foot_w/2.0+0.3,0.105]])
        self.foot_collision_box_4.InitFromBoxes(foot_collision_box_parameter,False)
        self.env.AddKinBody(self.foot_collision_box_4)
        self.foot_collision_box_4.SetTransform(out_of_env_transform)

        self.hand_collision_box_1 = rave.RaveCreateKinBody(self.env,'')
        self.hand_collision_box_1.SetName('hand_collision_box_1')
        hand_collision_box_parameter = np.array([[-0.105,0,0,0.10,self.hand_h/2.0,self.hand_w/2.0]])
        self.hand_collision_box_1.InitFromBoxes(hand_collision_box_parameter,False)
        self.env.AddKinBody(self.hand_collision_box_1)
        self.hand_collision_box_1.SetTransform(out_of_env_transform)

        self.hand_collision_box_2 = rave.RaveCreateKinBody(self.env,'')
        self.hand_collision_box_2.SetName('hand_collision_box_2')
        hand_collision_box_parameter = np.array([[-0.30,-self.hand_h/2.0,0,0.10,self.hand_h/2.0,self.hand_w*1.5]])
        self.hand_collision_box_2.InitFromBoxes(hand_collision_box_parameter,False)
        self.env.AddKinBody(self.hand_collision_box_2)
        self.hand_collision_box_2.SetTransform(out_of_env_transform)

        self.hand_collision_box_3 = rave.RaveCreateKinBody(self.env,'')
        self.hand_collision_box_3.SetName('hand_collision_box_3')
        hand_collision_box_parameter = np.array([[-0.30,self.hand_h/2.0,0,0.10,self.hand_h/2.0,self.hand_w*1.5]])
        self.hand_collision_box_3.InitFromBoxes(hand_collision_box_parameter,False)
        self.env.AddKinBody(self.hand_collision_box_3)
        self.hand_collision_box_3.SetTransform(out_of_env_transform)

    def move_body_to_collision_box_transform(self, collision_box_transform):

        affine_dofs = SE3_to_xyzrpy(np.dot(self.origin_body_transform, np.dot(collision_box_transform, self.inverse_body_collision_box_offset)))

        dof_values = self.robot.GetDOFValues()
        dof_values[self.DOFNameIndexDict['x_prismatic_joint']] = affine_dofs[0]
        dof_values[self.DOFNameIndexDict['y_prismatic_joint']] = affine_dofs[1]
        dof_values[self.DOFNameIndexDict['z_prismatic_joint']] = affine_dofs[2]
        dof_values[self.DOFNameIndexDict['roll_revolute_joint']] = affine_dofs[3] * DEG2RAD
        dof_values[self.DOFNameIndexDict['pitch_revolute_joint']] = affine_dofs[4] * DEG2RAD
        dof_values[self.DOFNameIndexDict['yaw_revolute_joint']] = affine_dofs[5] * DEG2RAD

        self.robot.SetDOFValues(dof_values)