#! /usr/bin/env/ python2

"""
    Description:    Velocity controller node for experiment with Mahsa
    Author:         Jesse Quattrociocchi
    Created:        May-July 2020
"""

# Imports for ROS side
import rospy
import numpy as np
import sys
import types
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PoseStamped
from tf.transformations import euler_from_quaternion
# Modules from this project
from grid_state_converter import *
import vel_control as vc
from laser_scan import Scan
from color_mixer import ColorMixer
# Imports for Algorithm side
import copy
import random
from partial_semantics import *
# Rviz
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Quaternion, Vector3


# ------------------ Start Class Definitions ------------------

class Scanner(Scan):
    ''' Converts a point cloud into a set of states that describe the state of a gridworld '''
    def __init__(self, scan_topic_name, grid_converter, debug=False):
        super(Scanner, self).__init__(scan_topic_name)
        self.grid_converter = grid_converter
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.debug = debug


    def convert_pointCloud_to_gridCloud(self, pc):
        ''' The input is a pointcloud2 generator, where each item is a tuple '''
        if not isinstance(pc, types.GeneratorType):
            print("The input must be a pointcloud2 generator (not the generator function)")
            sys.exit()
        states = set()
        for item in pc:
            if self.debug: print(item)
            new_pose = self.transform_coordinates(item)
            if self.debug: print(new_pose)
            if not (self.grid_converter.base.x < new_pose.pose.position.x < self.grid_converter.maximum.x) or not (self.grid_converter.base.y < new_pose.pose.position.y < self.grid_converter.maximum.y):
                continue
            else:
                pcPoint = Point()
                pcPoint.x = new_pose.pose.position.x
                pcPoint.y = new_pose.pose.position.y
                pcPoint.z = new_pose.pose.position.z
                states.add(self.grid_converter.cart2state(pcPoint))

        return states

    def transform_coordinates(self, coord, from_frame='base_scan', to_frame='odom'):
        ''' Gets frame transform at latest time '''
        p1 = PoseStamped()
        p1.header.frame_id = from_frame
        p1.pose.position.x = coord[0]
        p1.pose.position.y = coord[1] # Not sure this is right
        p1.pose.position.z = coord[2]
        p1.pose.orientation.w = 1.0    # Neutral orientation
        transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
        return tf2_geometry_msgs.do_transform_pose(p1, transform)

class BeliefMarker(object):
    def __init__(self):
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        rospy.sleep(0.5)
        self.marker = Marker()
        self.marker.header.frame_id = 'odom' # Change to odom for experiment
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.id = 0
        self.marker.type = Marker.CUBE_LIST
        self.marker.points = []
        self.marker.pose.orientation = Quaternion(0,0,0,1)
        self.marker.scale = Vector3(1,1,0.05)
        self.marker.colors = []
        self.marker.lifetime = rospy.Duration(0)

    def show_marker(self):
        self.pub.publish(self.marker)
        rospy.sleep(0.5)


# ------------------ End Class Definitions --------------------

# ------------------ Start Function Definitions ---------------

def wait_for_time():
    """Wait for simulated time to begin """
    while rospy.Time().now().to_sec() == 0:
        pass

def get_direction_from_key_stroke():
    while(True):
        val = raw_input("Enter command {a = left, w = up , s = down, d = right, h = hold, k = exit}: ")
        if val == 'w':
            print("You pressed up")
            return 'up'
        elif val == 's':
            print("You pressed down")
            return 'down'
        elif val == 'a':
            print("You pressed left")
            return 'left'
        elif val == 'd':
            print("You pressed right")
            return 'right'
        elif val == 'h':
            print("You pressed hold")
            return 'hold'
        elif val == 'k':
            print("You chose to exit")
            print("Closing program")
            sys.exit()
        else:
            print("You must enter a valid command {a = left, w = up , s = down, d = right, h = hold, k = exit}")

def next_waypoint_from_direction(direction, current_pose):
    """ Changes in x are from Left/Right action.
        CHanges in y are from Up/Down action.
        Set the next wp to the center of 1x1 cells (i.e x=1.5, y=1.5).
        Error logging uses ROS.
        direction is a string. current_pose is a Point object """

    wp = Point(current_pose.x, current_pose.y, None)
    if direction == 'up':
        wp.y = np.ceil(wp.y) + 0.5
    elif direction == 'down':
        wp.y = np.floor(wp.y) - 0.5
    elif direction == 'left':
        wp.x = np.floor(wp.x) - 0.5
    elif direction == 'right':
        wp.x = np.ceil(wp.x) + 0.5
    else:
        err_msg = "The direction value {} is not valid".format(direction)
        rospy.logerr(err_msg)
        sys.exit()

    return wp

def move_TB_keyboard(controller_object):
    """ Function to wrap up the process of moving the turtlebot via key strokes.
        Requires ROS to be running and a controller object """
    pose = Point(controller_object.x, controller_object.y, None)
    dir_val = get_direction_from_key_stroke()
    if dir_val == 'hold':
        print("* You chose to hold *")
    else:
        wp = next_waypoint_from_direction(dir_val, pose)
        controller_object.go_to_point(wp)

def move_TB(controller_object, dir_val):
    """ Function to wrap up the process of moving the turtlebot via input.
        Requires ROS to be running and a controller object """
    pose = Point(controller_object.x, controller_object.y, None)
    if dir_val == 'hold':
        print("* You chose to hold *")
    else:
        wp = next_waypoint_from_direction(dir_val, pose)
        controller_object.go_to_point(wp)

def make_grid_converter(grid_vars):
    ''' Returns a Grid class object '''
    base = Point()
    base.x = grid_vars[0]
    base.y = grid_vars[1]
    base.z = 0 # Used for the RVIZ marker
    maximum = Point()
    maximum.x = grid_vars[2]
    maximum.y = grid_vars[3]
    converter = Grid(grid_vars[5],grid_vars[4],base,maximum)
    return converter

def show_converter(controller):
    cart = Point()
    cart.x = controller.x
    cart.y = controller.y
    cart.z = 0
    print("You are now in State: {}".format(grid_converter.cart2state(cart)))



def get_visible_points_circle(center, radius):
    theta = np.linspace(0, 2*np.pi, 360)
    rays = np.linspace(0, radius, 35)
    vis_points = set()
    for angle in theta:
        for r in rays:
            x = center[0] + r*np.cos(angle)
            y = center[1] + r*np.sin(angle)
            vis_points.add((x,y))
    return vis_points

def get_vis_states_set(current_loc, converter, vis_dis=3.5):
    vis_points = get_visible_points_circle(current_loc, vis_dis)
    s = set()
    a_point = Point()
    for item in vis_points:
        if not (converter.base.x < item[0] < converter.maximum.x) or not (converter.base.y < item[1] < converter.maximum.y):
            continue
        else:
            a_point.x = item[0]
            a_point.y = item[1]
            a_point.z = 0
            s.add(converter.cart2state(a_point))

    return s

def get_occluded_states_set(controller, scanner, vis_dis=3.5):
    occl = set()
    temp = set()
    # Get all the points along a ray passing through each point in the pointcloud
    for item in scanner.pc_generator(field_names=('x', 'y', 'z','index')):
        angle = item[3]*scanner.raw.angle_increment
        radius = item[0]/np.cos(angle) # x/cos(theta)
        rays = np.linspace(radius, vis_dis, (vis_dis-radius)/0.1)
        for r in rays:
            x = r*np.cos(angle)
            y = r*np.sin(angle)
            temp.add((x,y,0)) # z is here for the transformation later on

    # Convert those points included in the set to the odom frame and make a new set
    for item in temp:
        new_pose = scanner.transform_coordinates(item)
        if not (scanner.grid_converter.base.x < new_pose.pose.position.x < scanner.grid_converter.maximum.x) or not (scanner.grid_converter.base.y < new_pose.pose.position.y < scanner.grid_converter.maximum.y):
            continue
        else:
            occlPoint = Point()
            occlPoint.x = new_pose.pose.position.x
            occlPoint.y = new_pose.pose.position.y
            occlPoint.z = new_pose.pose.position.z
            occl.add(scanner.grid_converter.cart2state(occlPoint))

    return occl

def make_array(scan, vis, occl, array_shape):
    ''' Assumes array_shape is (row,col).
        This does not check for overlapping states in the vis set and the scan set.
        It should work though bc of order. '''
    a = -np.ones(array_shape)
    real_occl = occl - occl.intersection(scan)
    real_vis = vis - vis.intersection(real_occl)

    for v in real_vis:
        row_ind = v / array_shape[1]
        col_ind = v % array_shape[1]
        a[row_ind,col_ind] = 0

    for s in scan:
        row_ind = s / array_shape[1]
        col_ind = s % array_shape[1]
        a[row_ind,col_ind] = 1

    return a

def make_user_wait(msg="Enter exit to exit"):
    data = raw_input(msg + "\n")
    if data == 'exit':
        sys.exit()

# ------------------ End Function Definitions -----------------

if __name__ == '__main__':
    rospy.init_node("velocity_controller")
    wait_for_time()

    # Some values to use for the Grid class that does conversions
    base_x = -6
    base_y = -4
    max_x = 6
    max_y = 4
    nb_y = 8
    nb_x = 12
    shape = (nb_y,nb_x)

    # make_user_wait('Press enter to start')

    # Create velocity controller and converter objects
    vel_controller = vc.VelocityController('/odom', '/cmd_vel')
    grid_vars = [base_x, base_y, max_x, max_y, nb_x, nb_y]
    grid_converter = make_grid_converter(grid_vars)

    # Set up and initialize RVIZ marker objects
    cm = ColorMixer('green', 'red')
    belief_marker = BeliefMarker()
    l = np.arange(0,nb_x*nb_y) # Specific to size of state space
    for item in l:
        p = grid_converter.state2cart(item)
        p.z = -0.05
        belief_marker.marker.points.append(p)
        belief_marker.marker.colors.append(ColorRGBA(0.0,0.0,0.0,1.0))

    belief_marker.show_marker()
    # Create scanner
    scanner = Scanner('/scan', grid_converter)
    print("Sleeping for 3 seconds to allow all ROS nodes to start")
    rospy.sleep(3)
    # # Set initial point
    # init_point = grid_converter.state2cart(0)
    # vel_controller.go_to_point(init_point)


    ############  The following is the code from Mahsa that runs her algorithm  ############


    # define simulation parameters
    n_iter = 1
    infqual_hist_all = []
    risk_hist_all = []
    timestep_all = []
    plan_count_all = []
    task_flag_all = []

    for iter in range(n_iter):

        # create problem setting
        model = MDP('gridworld', dim=(8,12), p_correctmove=0.95, init_state=0)
        # model.semantic_representation(prior_belief='exact')
        # print(model.label_true)
        obs_pos = [6,7,8,10,18,19,20,22,26,27,38,39,54,55,58,59,61,66,67,73,85,88,89,90,91]
        targ_pos = [95]
        model.semantic_representation(obstacle_pos=obs_pos, target_pos=targ_pos, prior_belief='random-obs')
        perc_flag = True
        bayes_flag = True
        replan_flag = True
        div_test_flag = True
        act_info_flag = True
        spec_true = [[],[]]
        for s in range(len(model.states)):
            if model.label_true[s,0] == True:
                spec_true[0].append(s)
            if model.label_true[s,1] == True:
                spec_true[1].append(s)

        visit_freq = np.ones(len(model.states))

    ##############################################################################

        # simulation results
        term_flag = False
        task_flag = False
        timestep = 0
        max_timestep = 100 # Increase if agent does not reach goal
        plan_count = 0
        div_thresh = 0.001
        n_sample = 10
        risk_thresh = 0.25
        state_hist = []
        state_hist.append(model.init_state)
        action_hist = [[],[]] # [[chosen action],[taken action]]
        infqual_hist = []
        infqual_hist.append(info_qual(model.label_belief))
        risk_hist = []

        while not term_flag:

            if perc_flag:
                # estimate map
                label_est = estimate_AP(model.label_belief, method='risk-averse')
                spec_est = [[],[]]
                for s in range(len(model.states)):
                    if label_est[s,0] == True:
                        spec_est[0].append(s)
                    if label_est[s,1] == True:
                        spec_est[1].append(s)
                # print("obstacle:   ",spec_est[0])
                # print("target:     ",spec_est[1])

            if replan_flag or (not replan_flag and plan_count==0):
                # find an optimal policy
                (vars_val, opt_policy) = verifier(copy.deepcopy(model), spec_est)
                print(opt_policy[49])
                plan_count += 1

            if act_info_flag:
                # risk evaluation
                prob_sat = stat_verifier(model,state_hist[-1],opt_policy,spec_est,n_sample)
                risk = np.abs(vars_val[state_hist[-1]] - prob_sat); print(vars_val[state_hist[-1]],prob_sat)
                risk_hist.append(risk)
                print("Risk due to Perception Uncertainty:   ",risk)

                # perception policy
                if risk > risk_thresh:
                    # implement perception policy
                    timestep += 1
                    state = state_hist[-1]
                    action = 0
                else:
                    pass
            timestep += 1
            print("Timestep:   ",timestep)
            state = state_hist[-1]
            opt_act = opt_policy[state]
            if 0 in opt_act and len(opt_act)>1:
                opt_act = opt_act[1:]

            action = np.random.choice(opt_act)

            action_hist[0].append(action)
            next_state = np.random.choice(model.states, p=model.transitions[state,action])
            # identify taken action
            for a in model.actions[model.enabled_actions[state]]:
                if model.action_effect(state,a)[0] == next_state:
                    action_taken = a
            action_hist[1].append(action_taken)
            state_hist.append(next_state)

    ############################################################################## uncommented for scenario 2
            # get new information
            # (obs,p_obs_model) = obs_modeling(model)
            #
            # # update belief
            # next_label_belief = belief_update(model.label_belief, obs,
            #                                  p_obs_model, bayes_flag)

    ############################################################################### commented for scenario 2

            # ----------------- Q --------------------------------------- 
            # array from Lidar : lid = np.zeros((dim1,dim2), dtype=np.float64) # {0=empty, -1=unseen, 1=obstacle}
            scan_states = scanner.convert_pointCloud_to_gridCloud(scanner.pc_generator())
            vis_states = get_vis_states_set((vel_controller.x, vel_controller.y), grid_converter)
            occluded_states = get_occluded_states_set(vel_controller, scanner) # Not clean but was faster to implement
            lid = make_array(scan_states, vis_states, occluded_states, shape)
            # print(lid)
            # ----------------- Q ---------------------------------------

            lid_adapted = np.reshape(lid, len(model.states))
            # update belief
            next_label_belief = copy.deepcopy(model.label_belief)
            visit_freq_next = copy.deepcopy(visit_freq) + 1
            for s in model.states:
                # update frequency
                if lid_adapted[s] == -1:
                    visit_freq_next[s] -= 1
                # update for 'obstacle'
                elif lid_adapted[s] == 0:
                    next_label_belief[s,0] = (next_label_belief[s,0]*visit_freq[s] + 0) / visit_freq_next[s]
                elif lid_adapted[s] == 1:
                    next_label_belief[s,0] = (next_label_belief[s,0]*visit_freq[s] + 1) / visit_freq_next[s]
                # # update for 'target'
                # elif lid_adapted[s] == 2:
                #     next_label_belief[s,1] = (next_label_belief[s,1]*visit_freq[s] + 1) / visit_freq_next[s]
                else:
                    raise NameError("Given value in the Lidar output is unaccepted")
            visit_freq = copy.deepcopy(visit_freq_next)

            # ----------------- Q ---------------------------------------
            ''' Set marker color based on belief '''
            belief_marker.marker.colors = []
            for s in model.states:
                # print(next_label_belief[s,0])
                cn = cm.get_color_norm(model.label_belief[s,0])
                belief_marker.marker.colors.append(ColorRGBA(cn[0], cn[1], cn[2], 1.0))
            belief_marker.show_marker()
            rospy.sleep(0.25)
            # ----------------- Q ---------------------------------------

            # ----------------- Q ---------------------------------------
            # move to next state. Use: action_hist[1][-1] # {0 : 'stop', 1 : 'up', 2 : 'right', 3 : 'down', 4 : 'left'}
            direction = {0 : 'hold', 1 : 'up', 2 : 'right', 3 : 'down', 4 : 'left'}
            print(action_hist[1][-1])
            print(direction[action_hist[1][-1]])
            print(state_hist[-1])
            move_TB(vel_controller, direction[action_hist[1][-1]])
            # make_user_wait()
            # ----------------- Q ---------------------------------------

            # divergence test on belief
            if div_test_flag:
                div = info_div(model.label_belief,next_label_belief)
                print("Belief Divergence:   ",div)
                if info_div(model.label_belief,next_label_belief) > div_thresh:
                    replan_flag = True
                else:
                    replan_flag = False
            model.label_belief = np.copy(next_label_belief)
            infqual_hist.append(info_qual(model.label_belief))

            # check task realization
            if model.label_true[state_hist[-1],0] == True:
                term_flag = True
                print("at a state with an obstacle")

            if model.label_true[state_hist[-1],1] == True:
                task_flag = True
                term_flag = True
                print("at a target state")

            if timestep == max_timestep:
                term_flag = True
                print("timestep exceeded the maximum limit")

        print("Number of Time Steps:   ",timestep)
        print("Number of Replanning:   ",plan_count)

        infqual_hist_all.append(infqual_hist)
        risk_hist_all.append(risk_hist)
        timestep_all.append(timestep)
        plan_count_all.append(plan_count)
        task_flag_all.append(int(task_flag))

    task_rate = np.mean(task_flag_all)

    ########################################################################################
    # The following are examples of how to use some of the features

    # # Some initial point to show to use the controller
    # init_point = Point(0.5, 0.5, None)
    # vel_controller.go_to_point(init_point)

    # while True:
    #     # # Ask the user for a cardinal direction to move the robot, and then move it
    #     # move_TB_keyboard(vel_controller)

    #     # # Show how the converter can be used
    #     # show_converter(vel_controller)

    #     # # Show how to convert scan to grid point and find states that are out of range
    #     scan_states = scanner.convert_pointCloud_to_gridCloud(scanner.pc_generator())
    #     vis_states = get_vis_states_set((vel_controller.x, vel_controller.y), grid_converter)

    #     array = make_array(scan_states, vis_states, shape)
    #     # print(array)

    #     make_user_wait()
