#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos

class LaserScanContainer:
    def __init__(self):
        self.inited = False
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.ranges = None
        self.cart = None


    def set_params(self, msg):
        self.angle_min = float(msg.angle_min)
        self.angle_max = float(msg.angle_max)
        self.angle_increment = float(msg.angle_increment)
        self.ranges = msg.ranges
        self.euc = None
        self.inited = True

    def convert_to_cartesian(self):
        max_r = max(self.ranges)
        self.cart = np.zeros((len(self.ranges), 2))
        for i, r in enumerate(self.ranges):
            phi = i*self.angle_increment + self.angle_min

            self.cart[i, 0] = r * cos(phi)
            self.cart[i, 1] = r * sin(phi)

            


def callback(msg):
    global scan 
    if not scan.inited:
        scan.set_params(msg)
        scan.convert_to_cartesian()
        plt.scatter(scan.cart[:, 0], scan.cart[:, 1])
        plt.savefig('tmp.png')
        

rospy.init_node('log')
scan = LaserScanContainer()
rospy.Subscriber('/base_scan', LaserScan, callback)

rospy.spin()
