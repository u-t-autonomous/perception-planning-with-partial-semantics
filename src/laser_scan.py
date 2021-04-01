#! /usr/bin/env/ python2

import rospy
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

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
