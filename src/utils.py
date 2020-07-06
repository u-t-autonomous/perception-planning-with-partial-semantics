#! /usr/bin/env/ python2

"""
    Description:    Velocity controller node for experiment with Mahsa
    Author:         Jesse Quattrociocchi
    Created:        May 2019
"""

# import rospy
import numpy as np

def make_array(some_set, array_shape):
    # Assumes array_shape is (row,col)
    a = np.zeros(array_shape)
    for s in some_set:
        row_ind = s / array_shape[1]
        col_ind = s % array_shape[1]
        a[row_ind,col_ind] = 1

    return a

def get_visible_points_circle(center, radius):
    theta = np.arange(0, 2*np.pi, (2*np.pi)/365)
    vis_points = set()
    for angle in theta:
        x = center[0] + radius*np.cos(angle)
        y = center[1] + radius*np.sin(angle)
        vis_points.add(x,y)
    return vis_points





if __name__ == '__main__':
    location = (0,0)
    vis_dis = 1
    visible_points = get_visible_points_circle(location, vis_dis)
    print(len(visible_points))
    print(visible_points)

