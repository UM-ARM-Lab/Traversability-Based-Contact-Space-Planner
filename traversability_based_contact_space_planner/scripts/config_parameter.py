import numpy as np

# manipulator indexing
manip_list = {0:'l_leg',1:'r_leg',2:'l_arm',3:'r_arm'}
manip_inverse_list = {'l_leg':0,'r_leg':1,'l_arm':2,'r_arm':3}
leg_manip_list = ['l_leg','r_leg']
arm_manip_list = ['l_arm','r_arm']

# global constant
mu = 0.5 # friction coefficient

# environment generation option
tilted_surface_probability = 0.5

# drawing object handle
draw_handles = []
contact_draw_handles = []

# planning parameter
draw_planning_progress = True
ANA_time_limit = 300.0
ANA_planning_time = 0.001
goal_br = 0.2
goal_hr = 0.82
dist_env_transition_cost_ratio = 10.0
dist_step_cost_ratio = 3.0
dist_angle_cost_ratio = 0.1
hand_body_cost_ratio = 0.5
motion_mode_changing_cost_ratio = 2.0

# grid resolution
map_grid_resolution = 0.135
foot_contact_point_resolution = 0.025

# motion mode
motion_mode_list = ['all_manipulators','legs_and_left_arm','legs_and_right_arm','legs_only']
motion_mode_name_index_dict = {name:index for index,name in enumerate(motion_mode_list)}
motion_mode_color_dict = {0:(1,0,0), 1:(0,0,1), 2:(1,1,0), 3:(0,1,0)}
traversability_regressor_mode_list = ['full_manipulators', 'legs_and_one_hand', 'legs_only']
traversability_regressor_mode_name_index_dict = {'full_manipulators':0, 'legs_and_one_hand':1, 'legs_only':3}

# generate trajectory
generate_trajectory = True

# utility constant
free_floating_pose = [-99.0,-99.0,-99.0,-99.0,-99.0,-99.0]
out_of_env_transform = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-99.0],[0,0,0,1]])

# data IO parameter
taking_planning_result_log = False
planning_data_path = 'escher_motion_planning_data/'

# motion plan generation and matching
min_plan_dist = 0.1
cg = 0.01
cs = 0.01
cm = 0.01
co = 0.01
