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
from geometry_msgs.msg import Twist, Point, PoseStamped, PoseWithCovarianceStamped
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from grid_state_converter import *
# Imports for Algorithm side
import copy
import random
from partial_semantics import *
# Rviz
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Quaternion, Vector3
from actionlib_msgs.msg import GoalStatusArray


# ------------------ Start Class Definitions ------------------

class VelocityController:
    """Simple velocity controller meant to be used with turtlebot3"""
    def __init__(self, odom_topic_name, cmd_vel_topic_name, debug=False):
        self.debug = debug
        self.__odom_sub = rospy.Subscriber(odom_topic_name, Odometry, self.__odomCB)
        self.__amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__amclCB)
        self.__goal_stat_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.__goalStatCB)
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_name, Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.x = None
        self.y = None
        self.yaw = None
        self.x_odom = None
        self.y_odom = None
        self.yaw_odom = None
        self.heading = None
        self.r = rospy.Rate(4)
        self.vel_cmd = Twist()
        self.goal_status = GoalStatusArray()

    def __odomCB(self, msg):
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self.yaw_odom = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def __amclCB(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def __goalStatCB(self, msg):
        self.goal_status = msg


    def go_to_point(self, goal):
        # input variable goal should be of type geometry_msgs/Point

        print("Starting to head towards the waypoint")

        ''' First do the rotation towards the goal '''
        error_x = goal.x - self.x_odom
        error_y = goal.y - self.y_odom
        angle_to_goal = np.arctan2(error_y, error_x)
        angle_error = self.yaw_odom - angle_to_goal

        if self.debug:
            print("Starting to rotate towards waypoint")

        while abs(angle_error) > 0.05:
            ''' Only useful if there is some slip/slide of the turtlebot while rotating '''
            # error_x = goal.x - self.x
            # error_y = goal.y - self.y
            # angle_to_goal = np.arctan2(error_y, error_x) # # #
            angle_error = self.yaw_odom - angle_to_goal
            if self.debug:
                print("Angle to goal: {:.5f},   Yaw: {:.5f},   Angle error: {:.5f}".format(angle_to_goal, self.yaw_odom, angle_error))
            if angle_to_goal >= 0:
                if self.yaw_odom <= angle_to_goal and self.yaw_odom >= angle_to_goal - np.pi:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = np.minimum(abs(angle_error), 0.4)
                else:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = -np.minimum(abs(angle_error), 0.4)
            else:
                if self.yaw_odom <= angle_to_goal + np.pi and self.yaw_odom > angle_to_goal:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = -np.minimum(abs(angle_error), 0.4)
                else:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = np.minimum(abs(angle_error), 0.4)
            # Publish and set loop rate
            self.cmd_vel_pub.publish(self.vel_cmd)
            self.r.sleep()
            # Calculate angle error again before loop condition is checked
            angle_error = self.yaw_odom - angle_to_goal

        # Stop rotation
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        if self.debug:
            print("Stopping the turn")


        # Move to the goal with nav stack
        self.getHeading([goal.x, goal.y])
        wp = PoseStamped()
        wp.header.frame_id = 'map'
        wp.pose.position.x = goal.x
        wp.pose.position.y = goal.y
        q = quaternion_from_euler(0, 0, self.heading)
        wp.pose.orientation.x = q[0]
        wp.pose.orientation.y = q[1]
        wp.pose.orientation.z = q[2]
        wp.pose.orientation.w = q[3]

        # Send goal to navstack
        goal_reached = False
        complete_counter = 0
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(30)
        self.goal_pub.publish(wp)
        # count = 0

        while not goal_reached:
            # count += 1
            # try:
            #     if count < 10:
            #         print(self.goal_status.status_list[-1].status)
            # except:
            #     print("Not printing the status")
            #     count = 0


            try:
                status_value = self.goal_status.status_list[-1].status
            except:
                status_value = 1
                complete_counter = 0

            if status_value == 1:
                complete_counter = 0
                continue
            elif status_value == 4:
                print("Status is 4: Waiting 2 seconds then sending the waypoiny again...")
                rospy.sleep(2.)
                complete_counter = 0
                self.goal_pub.publish(wp)
            elif status_value == 3:
                complete_counter += 1
                if complete_counter >= 80:
                    goal_reached = True
            else:
                rospy.logwarn("Goal status is: {}".format(self.goal_status.status_list[-1].status))

            if rospy.Time.now() > end_time:
                rospy.logwarn("The goal status loop has run for 30 seconds. Setting goal_reached = True")
                goal_reached = True

        print("** Waypoint Reached **")

    def getHeading(self, desiredState=np.array([5.0, 5.0])):
        relPos = desiredState - np.array([self.x, self.y])
        self.heading = np.arctan2(relPos[1], relPos[0])

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


class ColorMixer(object):
    ''' Finds the color mix between two colors.
        color_0 and color_1 are chosen from:
            {'red', 'green', 'blue'}. '''
    def __init__(self, color_0, color_1):
        colors = ('red', 'green', 'blue')
        if not color_0 in colors or not color_1 in colors:
            rospy.logerr("Choose a color from ('red', 'green', 'blue')")
            sys.exit()

        self._c0 = self.__determine_input(color_0)
        self._c1 = self.__determine_input(color_1)
        self.last_c = []

    ''' Input is a double on [0-1] where:
        0 maps to c_0 = 255 and c_1 = 0,
        1 maps to c_0 = 0 and c_1 = 255. '''
    def __determine_input(self, color):
        if color == 'red':
            return [255, 0, 0]
        elif color == 'green':
            return [0, 255, 0]
        elif color == 'blue':
            return [0, 0, 255]
        else:
            rospy.logerr("Choose a color from ('red', 'green', 'blue')")
            sys.exit()

    def __check_mixing_value(self, val):
        if not (0 <= val <= 1):
            rospy.logerr("get_color value must be between [0-1]")
            sys.exit()

    def get_color(self, val):
        self.__check_mixing_value(val)
        self.last_c = [val*(self._c1[j] - self._c0[j]) + self._c0[j] for j in range(3)]
        return self.last_c

    def get_color_norm(self, val):
        self.__check_mixing_value(val)
        self.last_c = [val*(self._c1[j] - self._c0[j]) + self._c0[j] for j in range(3)]
        self.last_c[:] = [x/255 for x in self.last_c]
        return self.last_c


class BeliefMarker(object):
    def __init__(self, grid_vars):
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        rospy.sleep(0.5)
        self.marker = Marker()
        self.marker.header.frame_id = 'odom' # Change to odom for experiment
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.id = 0
        self.marker.type = Marker.CUBE_LIST
        # self.marker.action = Marker.ADD   # Not sure what this is for???
        self.marker.points = []
        self.marker.pose.orientation = Quaternion(0,0,0,1)
        sx = ( abs(grid_vars[0]) + abs(grid_vars[2]) ) / grid_vars[4]
        sy = ( abs(grid_vars[1]) + abs(grid_vars[3]) ) / grid_vars[5]
        self.marker.scale = Vector3(sx,sy,0.05)
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
        Sets the next wp to the center of 1x1 cells (i.e x=1.5, y=1.5).
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

def next_waypoint_from_direction_v3(direction, current_pose, converter):
    """ Changes in x are from Left/Right action.
        CHanges in y are from Up/Down action.
        Sets the next wp to a discrete state.
        Error logging uses ROS.
        direction is a string. current_pose is a Point object """

    current_state = converter.cart2state(current_pose)
    # print("The current pose is set to: {}".format(current_pose))
    # print("The current state is set to: {}".format(current_state))
    if direction == 'up':
        current_state -= converter.col
    elif direction == 'down':
        current_state += converter.col
    elif direction == 'left':
        current_state -= 1
    elif direction == 'right':
        current_state += 1
    else:
        err_msg = "The direction value {} is not valid".format(direction)
        rospy.logerr(err_msg)
        sys.exit()

    # print("The goal state is now: {}".format(current_state))
    # print("The goal pose is now: {}".format(converter.state2cart(current_state)))
    return converter.state2cart(current_state)

def next_waypoint_from_direction_ft(direction, current_pose):
    """ Changes in x are from Left/Right action.
        Changes in y are from Up/Down action.
        Sets the next wp to the center of 1x1 cells (i.e x=1.5, y=1.5).
        Error logging uses ROS.
        direction is a string. current_pose is a Point object """
    feet = 0.3048

    wp = Point(current_pose.x/feet, current_pose.y/feet, None) #feet
    if direction == 'up':
        wp.y = np.ceil(wp.y)
        if wp.y % 2 == 0:
            wp.y += 2
            wp.x = np.round(wp.x)
        elif wp.y % 2 == 1:
            wp.y += 1
            wp.x = np.round(wp.x)
    elif direction == 'down':
        wp.y = np.floor(wp.y)
        if wp.y % 2 == 0:
            wp.y -= 2
            wp.x = np.round(wp.x)
        elif wp.y % 2 == 1:
            wp.y -= 1
            wp.x = np.round(wp.x)
    elif direction == 'left':
        wp.x = np.floor(wp.x)
        if wp.x % 2 == 0:
            wp.x -= 2
            wp.y = np.round(wp.y)
        elif wp.x % 2 == 1:
            wp.x -= 1
            wp.y = np.round(wp.y)
    elif direction == 'right':
        wp.x = np.ceil(wp.x)
        if wp.x % 2 == 0:
            wp.x += 2
            wp.y = np.round(wp.y)
        elif wp.x % 2 == 1:
            wp.x += 1
            wp.y = np.round(wp.y)
    else:
        err_msg = "The direction value {} is not valid".format(direction)
        rospy.logerr(err_msg)
        sys.exit()

    wpr = Point(wp.x*feet, wp.y*feet, None)
    return wpr

def move_TB_keyboard(controller_object, converter):
    """ Function to wrap up the process of moving the turtlebot via key strokes.
        Requires ROS to be running and a controller object """
    pose = Point(controller_object.x, controller_object.y, None)
    dir_val = get_direction_from_key_stroke()
    if dir_val == 'hold':
        print("* You chose to hold *")
    else:
        wp = next_waypoint_from_direction_v3(dir_val, pose, converter)
        controller_object.go_to_point(wp)

def move_TB(controller_object, dir_val):
    """ Function to wrap up the process of moving the turtlebot via input.
        Requires ROS to be running and a controller object """
    pose = Point(controller_object.x, controller_object.y, None)
    if dir_val == 'hold':
        print("* You chose to hold *")
    else:
        wp = next_waypoint_from_direction_v3(dir_val, pose, converter)
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

    # Some values to use for the Grid class that does conversions (Meters)
    base_x = -0.25
    base_y = -0.3
    max_x = 3.1
    max_y = 1.9
    nb_y = 4
    nb_x = 6
    shape = (nb_y,nb_x)

    # # Some values to use for the Grid class that does conversions (Meters)
    # base_x = -0.15
    # base_y = -0.15
    # max_x = 3.25
    # max_y = 2.3
    # nb_y = 8
    # nb_x = 12
    # shape = (nb_y,nb_x)

    # # Some values to use for the Grid class that does conversions (Meters)
    # base_x = -feet
    # base_y = -feet
    # max_x = (12*feet - feet)
    # max_y = (8*feet - feet)
    # nb_y = 4
    # nb_x = 6
    # shape = (nb_y,nb_x)

    # make_user_wait('Press enter to start')

    # Create velocity controller and converter objects
    vel_controller = VelocityController('/odom', '/cmd_vel')
    grid_vars = [base_x, base_y, max_x, max_y, nb_x, nb_y]
    grid_converter = make_grid_converter(grid_vars)

    # Set up and initialize RVIZ marker objects
    cm = ColorMixer('green', 'red')
    belief_marker = BeliefMarker(grid_vars)
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





    make_user_wait("Press Enter to Start")

    while True:
        # # Ask the user for a cardinal direction to move the robot, and then move it
        move_TB_keyboard(vel_controller, grid_converter)



        # # ----------------- Q --------------------------------------- 
        # # array from Lidar : lid = np.zeros((dim1,dim2), dtype=np.float64) # {0=empty, -1=unseen, 1=obstacle}
        scan_states = scanner.convert_pointCloud_to_gridCloud(scanner.pc_generator())
        vis_states = get_vis_states_set((vel_controller.x, vel_controller.y), grid_converter)
        occluded_states = get_occluded_states_set(vel_controller, scanner) # Not clean but was faster to implement
        lid = make_array(scan_states, vis_states, occluded_states, shape)
        print(lid)
        # # ----------------- Q ---------------------------------------


        # # ----------------- Q ---------------------------------------
        # ''' Set marker color based on belief '''
        # belief_marker.marker.colors = []
        # for s in model.states:
        #     # print(next_label_belief[s,0])
        #     cn = cm.get_color_norm(model.label_belief[s,0])
        #     belief_marker.marker.colors.append(ColorRGBA(cn[0], cn[1], cn[2], 1.0))
        # belief_marker.show_marker()
        # rospy.sleep(0.25)
        # # ----------------- Q ---------------------------------------


        # # ----------------- Q ---------------------------------------
        # # move to next state. Use: action_hist[1][-1] # {0 : 'stop', 1 : 'up', 2 : 'right', 3 : 'down', 4 : 'left'}
        # direction = {0 : 'hold', 1 : 'up', 2 : 'right', 3 : 'down', 4 : 'left'}
        # print(action_hist[1][-1])
        # print(direction[action_hist[1][-1]])
        # print(state_hist[-1])
        # move_TB(vel_controller, direction[action_hist[1][-1]])
        # # make_user_wait()
        # # ----------------- Q ---------------------------------------








        # # Show how the converter can be used
        # show_converter(vel_controller)

        # # # Show how to convert scan to grid point and find states that are out of range
        # scan_states = scanner.convert_pointCloud_to_gridCloud(scanner.pc_generator())
        # vis_states = get_vis_states_set((vel_controller.x, vel_controller.y), grid_converter)
        # occluded_states = get_occluded_states_set(vel_controller, scanner) # Not clean but was faster to implement
        # array = make_array(scan_states, vis_states, occluded_states, shape)
        # print(array)

        belief_marker.marker.colors = []
        for r in lid:
            for s in r:
                if s != 1:
                    cn = cm.get_color_norm(0)
                else:
                    cn = cm.get_color_norm(1)
                belief_marker.marker.colors.append(ColorRGBA(cn[0], cn[1], cn[2], 1.0))
        belief_marker.show_marker()
        rospy.sleep(0.25)

        make_user_wait("Enter to proceed")































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
