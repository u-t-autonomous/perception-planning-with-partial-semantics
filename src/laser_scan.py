#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from visual_slam.msg import ObjectLocation, ObjectLocations

class ClassifiedScan(object):
    ''' Scan object that holds the laser scan and point cloud of the input topic '''
    def __init__(self, scan_data, name, confidence):
        self.raw = scan_data # scan_data is of type LaserScan
        self.lp = lg.LaserProjection()
        self.cloud = self.lp.projectLaser(scan_data)
        self.name = name
        self.confidence = confidence
        # self.__scan_sub = rospy.Subscriber(scan_topic_name, LaserScan, self.__scanCB)

    # def __scanCB(self, msg):
    #    self.raw = msg
    #    self.cloud = self.lp.projectLaser(msg)

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

class ScanList(object):
    def __init__(self, scan_topic_name):
        self.__sub = rospy.Subscriber(scan_topic_name, ObjectLocations, self.__callback)
        self.data = list() # List of ClassifiedScan objects

    def __callback(self, msg):
        self.data = list()
        if len(msg.obj_locations) > 0:
            for obstacle in msg.obj_locations:
                scanner = ClassifiedScan(obstacle.scan_data, obstacle.object_class, obstacle.confidence)
                self.data.append(scanner)
        else:
            empty_scan_data = LaserScan()
            empty_scanner = ClassifiedScan(empty_scan_data, "N/A", 0.0)
            self.data.append(empty_scanner)

    def print_scan(self, scan_type='cloud'):
        for scanner in self.data:
            scanner.print_scan(scan_type)
            print("\n")

    def pc_generator(self, field_names=('x', 'y', 'z')):
        output = list()
        for scanner in self.data:
            output.append(scanner.pc_generator(field_names))
        return output

    def get_names_list(self):
        names = list()
        for scanner in self.data:
            names.append(scanner.name)
        return names

    def get_confidences_list(self):
        confidences = list()
        for scanner in self.data:
            confidences.append(scanner.confidence)
        return confidences
