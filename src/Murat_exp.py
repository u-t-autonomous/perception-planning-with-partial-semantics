#! /usr/bin/env/ python2

"""
    Description:    Velocity controller NEEDS TO BE CLEANED UP!!!!
    Author:         Jesse Quattrociocchi
    Created:        May 2019
"""

import rospy
import numpy as np
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion

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

        print("Starting to head towards the waypoint")

        ''' First do the rotation towards the goal '''
        error_x = goal.x - self.x
        error_y = goal.y - self.y
        angle_to_goal = np.arctan2(error_y, error_x)
        angle_error = self.yaw - angle_to_goal

        if self.debug:
            print("Starting to rotate towards waypoint")

        while abs(angle_error) > 0.05:
            ''' Only useful if there is some slip/slide of the turtlebot while rotating '''
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
        distance_error = np.sqrt(error_x**2 + error_y**2)
        angle_to_goal = np.arctan2(error_y, error_x)
        angle_error = abs(self.yaw - angle_to_goal)
        previous_distance_error = 0
        total_distance_error = 0
        previous_angle_error = 0
        total_angle_error = 0

        kp_distance = 1
        ki_distance = 0.1
        kd_distance = 0.1

        kp_angle = 1
        ki_angle = 0.1
        kd_angle = 0.1

        if self.debug:
            print("Starting the PID")

        while distance_error > 0.05:
            error_x = goal.x - self.x
            error_y = goal.y - self.y
            distance_error = np.sqrt(error_x**2 + error_y**2)
            angle_to_goal = np.arctan2(error_y, error_x)
            angle_error = abs(self.yaw - angle_to_goal)

            total_distance_error = total_distance_error + distance_error
            total_angle_error = total_angle_error + angle_error
            diff_distance_error = distance_error - previous_distance_error
            diff_angle_error = angle_error - previous_angle_error

            control_signal_distance = kp_distance*distance_error + ki_distance*total_distance_error + kd_distance*diff_distance_error
            control_signal_angle = kp_angle*angle_error + ki_angle*total_angle_error + kd_angle*diff_angle_error

            self.vel_cmd.linear.x = np.minimum(control_signal_distance, 0.1)
            self.vel_cmd.angular.z = control_signal_angle

            if angle_to_goal >= 0:
                if self.yaw <= angle_to_goal and self.yaw >= angle_to_goal - np.pi:
                    self.vel_cmd.angular.z = np.minimum(abs(control_signal_angle), 0.4)
                else:
                    self.vel_cmd.angular.z = -np.minimum(abs(control_signal_angle), 0.4)
            else:
                if self.yaw <= angle_to_goal + np.pi and self.yaw > angle_to_goal:
                    self.vel_cmd.angular.z = -np.minimum(abs(control_signal_angle), 0.4)
                else:
                    self.vel_cmd.angular.z = np.minimum(abs(control_signal_angle), 0.4)

            previous_distance_error = distance_error
            previous_angle_error = angle_error

            self.cmd_vel_pub.publish(self.vel_cmd)
            self.r.sleep()

        # Stop motion
        self.cmd_vel_pub.publish(Twist())
        if self.debug:
            print("Stopping PID")
            print("Position is currently: ({:.5f},{:.5f})    Yaw is currently: [{:.5f}]".format(self.x, self.y, self.yaw))

        print("** Waypoint Reached **")


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
    """ Function to wrap up the process of moving the turtlebot via directional input.
        Requires ROS to be running and a controller object """
    pose = Point(controller_object.x, controller_object.y, None)
    if dir_val == 'hold':
        print("* You chose to hold *")
    else:
        wp = next_waypoint_from_direction(dir_val, pose)
        controller_object.go_to_point(wp)


def wait_for_time():
    """Wait for simulated time to begin """
    while rospy.Time().now().to_sec() == 0:
        pass

def make_user_wait(msg="Enter exit to exit"):
    data = raw_input(msg + "\n")
    if data == 'exit':
        sys.exit()

def make_wp_list_High():
    x_list = [2.6525,
            2.67875346284453,
            2.65331474331459,
            2.57047196713676,
            2.40801314960306,
            2.23968452997956,
            2.06608339400678,
            1.88759829101984,
            1.70620430665531,
            1.52185117711799,
            1.3352302117815,
            1.14774242304818,
            1.04067799435136,
            0.944762662402855,
            0.824732030813497,
            0.653278737426688,
            0.481472460229155,
            0.308114632159632,
            0.13167338842794,
            -0.049277878252393,
            -0.234712337348597]

    y_list = [0.179285714285714,
            0.341141110574587,
            0.505454111880492,
            0.670957982142299,
            0.836227395717013,
            0.975549479403729,
            1.09660483153186,
            1.13441655505261,
            1.17296407880009,
            1.18191548191398,
            1.13236199756972,
            0.904267190354389,
            0.668834247016701,
            0.428104422572631,
            0.243756926779694,
            0.179401118270869,
            0.164711559813752,
            0.212485122418645,
            0.316009224917304,
            0.416283796302494,
            0.513153263779824]

def make_wp_list_Low():
    y_list = [0.179285714285714,
            0.312258247772087,
            0.44307918831025,
            0.56528881223938,
            0.665570833248907,
            0.747690556261641,
            0.812816628227167,
            0.861064165686563,
            0.890584173024566,
            0.897989758857161,
            0.879465457795722,
            0.845902776184196,
            0.801602692670421,
            0.75001985377109,
            0.692687711814254,
            0.64055391561921,
            0.597707164636305,
            0.561753150714097,
            0.528980490512831,
            0.496475749112784,
            0.513566309520404]

    x_list = [2.6525,
            2.58233825008007,
            2.49647982324842,
            2.38403729167928,
            2.26006205527138,
            2.12711361467728,
            1.98677612875823,
            1.84011800246614,
            1.68736555685934,
            1.52906613848198,
            1.37205221280241,
            1.22033496705239,
            1.0728052934527,
            0.924903171436331,
            0.773205538866889,
            0.607039625644591,
            0.440897491858864,
            0.276406371541826,
            0.113562346221348,
            -0.04838004402777,
            -0.23498973501296]

def make_wp_list_Lowest():
    x_list = [2.6525,
            2.54949546174022,
            2.43883565231725,
            2.32129054766486,
            2.18746505210324,
            2.04875613336375,
            1.90584292278668,
            1.75961567708565,
            1.61062036786736,
            1.45948672598342,
            1.30699090849524,
            1.15319582659481,
            0.998215197092535,
            0.842081888966355,
            0.684896989165154,
            0.526633712673391,
            0.373361422895025,
            0.220228341138568,
            0.06747251126609,
            -0.084476682872025,
            -0.234992856799265]

    y_list = [0.179285714285714,
            0.283295954097603,
            0.380064807254564,
            0.470216364722557,
            0.54311535620045,
            0.606554427340453,
            0.660220954434946,
            0.704235609042585,
            0.738616331945364,
            0.763105150623263,
            0.778600196776769,
            0.785558777918163,
            0.784221259888349,
            0.774632265557078,
            0.75695505437567,
            0.73163326891972,
            0.702030380377592,
            0.665068193046082,
            0.62049850938055,
            0.568399385933984,
            0.513569844313991]

    waypoints = []
    for a,b in zip(x_list,y_list):
        waypoints.append(Point(a,b,None))

    return waypoints

''' You can run this file to test '''
if __name__ == '__main__':
    rospy.init_node("velocity_controller")
    wait_for_time()

    
    vel_controller = VelocityController('/odom', '/cmd_vel')
    print("Sleeping for 3 seconds to allow all ROS nodes to start")
    rospy.sleep(3)

    # init_point = Point(0.0, 0.0, None)
    # vel_controller.go_to_point(init_point)

    make_user_wait("Press Enter to Start")
    # waypoints = make_wp_list_High()
    # waypoints = make_wp_list_Low()
    waypoints = make_wp_list_Lowest()
    reached = 1
    for wp in waypoints:
        vel_controller.go_to_point(wp)
        rospy.loginfo("Turtlebot has reached waypoint {}".format(reached))
        reached += 1

    print("*--* Trajectory completed *--*")
