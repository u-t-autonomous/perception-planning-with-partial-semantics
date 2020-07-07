#! /usr/bin/env/ python2

"""
    Description:    Velocity controller node for experiment with Mahsa
    Author:         Jesse Quattrociocchi
    Created:        May-July 2019
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
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2
from tf.transformations import euler_from_quaternion
from grid_state_converter import *
# Imports for Algorithm side
import copy
import random
from partial_semantics import *


# ------------------ Start Class Definitions ------------------

class VelocityController:
    """Simple velocity controller meant to be used with turtlebot3"""
    def __init__(self, odom_topic_name, cmd_vel_topic_name, debug=False):
        self.debug = debug
        self.__odom_sub = rospy.Subscriber(odom_topic_name, Odometry, self.__odomCB)
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_name, Twist, queue_size = 1)

        self.x = None
        self.y = None
        self.yaw = None
        self.r = rospy.Rate(4)
        self.vel_cmd = Twist()

    def __odomCB(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def go_to_point(self, goal):
        # input variable goal should be of type geometry_msgs/Point

        print("Starting to head towards the goal")

        ''' First do the rotation towards the goal '''
        error_x = goal.x - self.x
        error_y = goal.y - self.y
        angle_to_goal = np.arctan2(error_y, error_x)
        angle_error = self.yaw - angle_to_goal

        if self.debug:
            print("Starting to turn towards goal")

        while abs(angle_error) > 0.05:
            # # Only useful if there is some slip/slide of the turtlebot while rotating # # #
            # error_x = goal.x - self.x
            # error_y = goal.y - self.y
            # angle_to_goal = np.arctan2(error_y, error_x) # # #
            angle_error = self.yaw - angle_to_goal
            if self.debug:
                print("Angle to goal: {:.5f},   Yaw: {:.5f},   Angle error: {:.5f}".format(angle_to_goal, self.yaw, angle_error))
            if angle_to_goal >= 0:
                if self.yaw <= angle_to_goal and self.yaw >= angle_to_goal - np.pi:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = np.minimum(abs(angle_error), 0.4)
                else:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = -np.minimum(abs(angle_error), 0.4)
            else:
                if self.yaw <= angle_to_goal + np.pi and self.yaw > angle_to_goal:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = -np.minimum(abs(angle_error), 0.4)
                else:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = np.minimum(abs(angle_error), 0.4)
            # Publish and set loop rate
            self.cmd_vel_pub.publish(self.vel_cmd)
            self.r.sleep()
            # Calculate angle error again before loop condition is checked
            angle_error = self.yaw - angle_to_goal

        # Stop rotation
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        if self.debug:
            print("Stopping the turn")

        ''' Then use a PID that controls the cmd velocity and drives the distance error to zero '''
        error_x = goal.x - self.x
        error_y = goal.y - self.y
        angle_to_goal = np.arctan2(error_y, error_x)
        last_rotation = 0
        goal_distance = np.sqrt(error_x**2 + error_y**2)
        distance = goal_distance
        previous_distance = 0
        total_distance = 0
        previous_angle = 0
        total_angle = 0

        kp_distance = 1
        ki_distance = 0.1
        kd_distance = 0.1

        kp_angle = 1
        ki_angle = 0.1
        kd_angle = 0.1

        if self.debug:
            print("Starting the PID")

        while distance > 0.05:
            y_start = self.y
            x_start = self.x
            rotation = self.yaw
            error_x = goal.x - x_start
            error_y = goal.y - y_start
            angle_to_goal = np.arctan2(error_y, error_x)
            if self.debug:
                print("BEFORE -- goal_z: {:.5f},  rotation: {:.5f}".format(angle_to_goal, rotation))
            if angle_to_goal < -np.pi/4 or angle_to_goal > np.pi/4:
                if goal.y < 0 and y_start < goal.y:
                    angle_to_goal = -2*np.pi + angle_to_goal
                elif goal.y >= 0 and y_start > goal.y:
                    angle_to_goal = 2*np.pi + angle_to_goal
            if last_rotation > np.pi - 0.1 and rotation <= 0:
                rotation = 2*np.pi + rotation
            elif last_rotation < -np.pi + 0.1 and rotation > 0:
                rotation = -2*np.pi + rotation
            if self.debug:
                print("AFTER -- goal_z: {:.5f},  rotation: {:.5f}".format(angle_to_goal, rotation))
            diff_angle = angle_to_goal - previous_angle
            diff_distance = distance - previous_distance

            distance = np.sqrt(error_x**2 + error_y**2)

            control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

            control_signal_angle = kp_angle*angle_to_goal + ki_angle*total_angle + kd_angle*diff_angle

            self.vel_cmd.angular.z = control_signal_angle - rotation
            self.vel_cmd.linear.x = np.minimum(control_signal_distance, 0.1)

            if self.vel_cmd.angular.z > 0:
                self.vel_cmd.angular.z = np.minimum(self.vel_cmd.angular.z, 1.0)
            else:
                self.vel_cmd.angular.z = np.maximum(self.vel_cmd.angular.z, -1.0)

            last_rotation = rotation
            self.cmd_vel_pub.publish(self.vel_cmd)
            previous_distance = distance
            previous_angle = angle_to_goal
            total_distance = total_distance + distance
            self.r.sleep()

        # Stop motion
        self.cmd_vel_pub.publish(Twist())
        if self.debug:
            print("Stopping PID")
            print("Position is currently: ({:.5f},{:.5f})    Yaw is currently: [{:.5f}]".format(self.x, self.y, self.yaw))

        print("** Goal Reached **")

class Scan(object):
    ''' Scan object that holds the laser scan and point cloud of the input topic '''
    def __init__(self, scan_topic_name):
        self.raw = LaserScan()
        self.cloud = None
        self.lp = lg.LaserProjection()
        self.__scan_sub = rospy.Subscriber(scan_topic_name, LaserScan, self.__scanCB)

    def __scanCB(self, msg):
        self.raw = msg
        self.cloud = self.lp.projectLaser(msg)

    def print_scan(self, scan_type='cloud'):
        if not isinstance(scan_type, str):
            print("The scan_type variable must be a string (raw, cloud)")
        if scan_type == 'cloud':
            print("The point cloud is: \n{}".format(self.cloud))
        elif scan_type == 'raw':
            print("the raw message is: \n{}".format(self.raw))
        else:
            print("The scan types are: raw, cloud")

    def pc_generator(self, field_names=('x', 'y', 'z')):
        return pc2.read_points(self.cloud, field_names)


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

# ------------------ End Class Definitions --------------------

# ------------------ Start Function Definitions ---------------

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
    base.z = 0 # Not used, but converter may require it to exist
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

def make_user_wait():
    data = raw_input("Enter exit to exit\n")
    if data == 'exit':
        sys.exit()


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

# ------------------ End Function Definitions -----------------

if __name__ == '__main__':
    rospy.init_node("velocity_controller")

    # Some values to use for the Grid class that does conversions
    base_x = -5
    base_y = -5
    max_x = 5
    max_y = 5
    nb_y = 10
    nb_x = 10
    shape = (nb_y,nb_x)

    # Set up vars and create objects
    vel_controller = VelocityController('/odom', '/cmd_vel')
    grid_vars = [base_x, base_y, max_x, max_y, nb_x, nb_y]
    grid_converter = make_grid_converter(grid_vars)
    scanner = Scanner('/scan', grid_converter)
    print("Sleeping for 3 seconds to allow all ROS nodes to start")
    rospy.sleep(3)

    init_point = Point(0.5, 0.5, None)
    vel_controller.go_to_point(init_point)


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
        model = MDP('gridworld')
        model.semantic_representation(prior_belief='noisy-ind') # changed for scenario 2
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
        max_timestep = 250
        plan_count = 0
        div_thresh = 0.001
        n_sample = 10
        risk_thresh = 0.1
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
                # print(opt_policy[0:20])
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
            print(lid)
            # ----------------- Q ---------------------------------------

            lid_adapted = np.reshape(lid, len(model.states))
            # update belief
            next_label_belief = copy.deepcopy(model.label_belief)
            visit_freq_next = copy.deepcopy(visit_freq) + 1
            for s in model.states:
                # update frequency
                if lid_adapted[s] == -1:
                    visit_freq_next[s] -= 1
                elif lid_adapted[s] == 0:
                    pass
                # update for 'obstacle'
                elif lid_adapted[s] == 1:
                    next_label_belief[s,0] = (next_label_belief[s,0]*visit_freq[s] + 1) / visit_freq_next[s]
                # # update for 'target'
                # elif lid_adapted[s] == 2:
                #     next_label_belief[s,1] = (next_label_belief[s,1]*visit_freq[s] + 1) / visit_freq_next[s]
                else:
                    raise NameError("Given value in the Lidar output is unaccepted")
            visit_freq = copy.deepcopy(visit_freq_next)

            # ----------------- Q --------------------------------------- 
            # move to next state. Use: action_hist[1][-1] # {0 : 'stop', 1 : 'up', 2 : 'right', 3 : 'down', 4 : 'left'}
            direction = {0 : 'hold', 1 : 'up', 2 : 'right', 3 : 'down', 4 : 'left'}
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
