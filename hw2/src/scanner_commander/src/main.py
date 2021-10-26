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
        self.filter_mask = None

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

        mask = self.filter_mask if self.filter_mask is not None else [True]*len(self.ranges)

        for i, (r, m) in enumerate(zip(self.ranges, mask)):
            if m:
                phi = i*self.angle_increment + self.angle_min

                self.cart[i, 0] = r * cos(phi)
                self.cart[i, 1] = r * sin(phi)

    def create_filter_mask(self, treshold=0.1):
        self.filter_mask = [True]
        for cur_id in range(1, len(self.ranges)-1):
            prev, cur, nxt = self.ranges[cur_id-1:cur_id+2]
            mean = (nxt + prev)/2
            prev_dif = cur - prev
            nxt_dif  = nxt - cur
            ratio = abs(prev_dif - nxt_dif)/mean

            self.filter_mask.append(int(ratio < treshold))
        self.filter_mask.append(True)

    def save_img(self, name):
        plt.scatter(self.cart[:, 0], self.cart[:, 1])
        plt.savefig(f'{name}.png')
        


def callback(msg):
    global scan 
    if not scan.inited:
        scan.set_params(msg)
        scan.convert_to_cartesian()
        scan.save_img('tmp')

        scan.create_filter_mask()
        scan.convert_to_cartesian()
        scan.save_img('tmp_filtered')
        
        


rospy.init_node('log')
scan = LaserScanContainer()
rospy.Subscriber('/base_scan', LaserScan, callback)

rospy.spin()
