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
    def __init__(self, odom_topic_name, cmd_vel_topic_name):
        
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
        # input variable point should be of type geometry_msgs/Point
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
            self.r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance

        # Stop motion
        self.cmd_vel_pub.publish(Twist())
        if self.debug:
            print("Stopping PID")

        print("Position is currently: ({:.5f},{:.5f})    Yaw is currently: [{:.5f}]".format(self.x, self.y, self.yaw))
        print("** Goal Reached **")

        ''' Previous implementations to move the turtlebot '''
        ''' ---------------------------------------------- '''
        # goal_reached = False

        # while not goal_reached:
        #     # Calculate errors
        #     error_x = goal.x - self.x
        #     error_y = goal.y - self.y
        #     angle_to_goal = np.arctan2(error_y, error_x)
        #     angle_error = self.yaw - angle_to_goal
        #     # angle_error = self.yaw - angle_to_goal
        #     d = np.sqrt(error_x**2 + error_y**2)
        #     # print("Angle error: {:.5f}, Distance error: {:.5f}".format(angle_error, d))
        #     print("Angle to goal: {:.5f},   Yaw: {:.5f},   Angle error: {:.5f}".format(angle_to_goal, self.yaw, angle_error))
        #     # Set message value
        #     if abs(angle_error) > 0.2:
        #         if angle_to_goal >= 0:
        #             if self.yaw <= angle_to_goal and self.yaw >= angle_to_goal - np.pi:
        #                 self.vel_cmd.linear.x = 0.0
        #                 self.vel_cmd.angular.z = np.minimum(abs(angle_error), 0.4)
        #             else:
        #                 self.vel_cmd.linear.x = 0.0
        #                 self.vel_cmd.angular.z = -np.minimum(abs(angle_error), 0.4)
        #         else:
        #             if self.yaw <= angle_to_goal + np.pi and self.yaw > angle_to_goal:
        #                 self.vel_cmd.linear.x = 0.0
        #                 self.vel_cmd.angular.z = -np.minimum(abs(angle_error), 0.4)
        #             else:
        #                 self.vel_cmd.linear.x = 0.0
        #                 self.vel_cmd.angular.z = np.minimum(abs(angle_error), 0.4)
        #     elif d > 0.05:
        #         self.vel_cmd.linear.x = np.minimum(d, 0.15)
        #         self.vel_cmd.angular.z = 0.0
        #     else:
        #         goal_reached = True
        #         self.vel_cmd.linear.x = 0.0
        #         self.vel_cmd.angular.z = 0.0



        #     # # Set message value
        #     # if angle_error >= 0.1:
        #     #     self.vel_cmd.linear.x = 0.0
        #     #     self.vel_cmd.angular.z = np.minimum(abs(angle_error), 0.5)
        #     # elif angle_error <= -0.1:
        #     #     self.vel_cmd.linear.x = 0.0
        #     #     self.vel_cmd.angular.z = -np.minimum(abs(angle_error), 0.5)
        #     # elif d > 0.05:
        #     #     self.vel_cmd.linear.x = np.minimum(d, 0.2)
        #     #     self.vel_cmd.angular.z = 0.0
        #     # else:
        #     #     goal_reached = True
        #     #     self.vel_cmd.linear.x = 0.0
        #     #     self.vel_cmd.angular.z = 0.0



        #     # Publish and set loop rate
        #     self.cmd_vel_pub.publish(self.vel_cmd)
        #     self.r.sleep()

        # print("Goal Reached")
        ''' ---------------------------------------------- '''

def get_direction_from_key_stroke():
    while(True):
        val = raw_input("Enter command {a = left, w = up , s = down, d = right, k = exit}: ")
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
        elif val == 'k':
            print("You chose to exit")
            print("Closing program")
            sys.exit()
        else:
            print("You must enter a valid command {a = left, w = up , s = down, d = right, k = exit}")

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

def move_TB(controller_object):
    """ Function to wrap up the process of moving the turtlebot via key strokes.
        Requires ROS to be running and a controller object """
    pose = Point(controller_object.x, controller_object.y, None)
    dir_val = get_direction_from_key_stroke()
    wp = next_waypoint_from_direction(dir_val, pose)
    controller_object.go_to_point(wp)



''' You can run this file to test '''
if __name__ == '__main__':
    rospy.init_node("velocity_controller")

    base_x = -5
    base_y = -5
    max_x = 5
    max_y = 5
    nb_y = 10
    nb_x = 10

    # Set up vars and create controller object
    grid_vars = [base_x, base_y, max_x, max_y, nb_x, nb_y]
    converter = make_converter(grid_vars)
    scanner = Scan("/scan")
    
    vel_controller = VelocityController('/odom', '/cmd_vel')
    print("Sleeping for 3 seconds to allow all ROS nodes to start")
    rospy.sleep(3)

    init_point = Point(0.5, 0.5, None)
    vel_controller.go_to_point(init_point)

    while True:
        move_TB(vel_controller)
        cart = Point()
        cart.x = vel_controller.x
        cart.y = vel_controller.y
        cart.z = 0
        print("You are now in State: {}".format(converter.cart2state(cart)))
        print("Select another point or exit")
