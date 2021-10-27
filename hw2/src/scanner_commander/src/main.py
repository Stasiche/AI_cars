#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
from time import sleep
from nav_msgs.msg import OccupancyGrid, MapMetaData


class LaserScanContainer:
    def __init__(self, filter_treshold, resolution, map_size):
        self.angle_min = None
        self.angle_increment = None
        self.ranges = None
        self.cart = None
        self.filter_mask = None
        self.grid_msg = None

        self.filter_treshold = filter_treshold
        self.resolution = resolution
        self.map_size = map_size
        self.map_side = int(map_size/2)
        self.grid_size = int(map_size / resolution)

        rospy.Subscriber('/base_scan', LaserScan, self.process_scan)
        self.map_publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=10)


    def set_params(self, msg):
        self.angle_min = float(msg.angle_min)
        self.angle_increment = float(msg.angle_increment)
        self.ranges = msg.ranges
        self.cart = None
        self.filter_mask = [True]*len(self.ranges)
    
    def init_grid(self):
        self.grid_msg = OccupancyGrid()
        self.grid_msg.info = MapMetaData()

        self.grid_msg.header.frame_id = "base_laser_link"
        self.grid_msg.info.resolution = self.resolution

        self.grid_msg.info.width = self.grid_size
        self.grid_msg.info.height = self.grid_size

        self.grid_msg.info.origin.position.x = -self.map_side
        self.grid_msg.info.origin.position.y = -self.map_side
        self.grid_msg.info.origin.position.z = 0

    def convert_to_cartesian(self):
        max_r = max(self.ranges)
        self.cart = np.zeros((len(self.ranges), 2))

        # TODO vectorize
        for i, (r, m) in enumerate(zip(self.ranges, self.filter_mask)):
            if m:
                phi = i*self.angle_increment + self.angle_min

                self.cart[i, 0] = r * cos(phi)
                self.cart[i, 1] = r * sin(phi)

    def create_filter_mask(self):
        self.filter_mask = [True]
        for cur_id in range(1, len(self.ranges)-1):
            prev, cur, nxt = self.ranges[cur_id-1:cur_id+2]
            mean = (nxt + prev)/2
            prev_dif = cur - prev
            nxt_dif  = nxt - cur
            ratio = abs(prev_dif - nxt_dif)/mean

            self.filter_mask.append(ratio < self.filter_treshold)
        self.filter_mask.append(True)

    def generate_map_data(self):
        map_data = np.zeros((self.grid_size, self.grid_size))

        for x, y in self.cart:
            if (abs(x) < self.map_side) and (abs(y) < self.map_side):
                i = int((x + self.map_side) / self.resolution)
                j = int((y + self.map_side) / self.resolution)

                map_data[j, i] += 1
                # map_data[j, i] += 1/(x**2 + y**2) if bool(x+y) else 0
        # increase contrast
        map_data = np.log(map_data)
        return (100*(map_data/map_data.max())).astype(np.int8).ravel()

    def process_scan(self, msg):
        self.set_params(msg)
        self.init_grid()
        self.create_filter_mask()
        self.convert_to_cartesian()
        self.grid_msg.data = self.generate_map_data()
        self.map_publisher.publish(self.grid_msg)


rospy.init_node('map_generator')
# LaserScanContainer(1e-2, 0.1, 20)
LaserScanContainer(1e-2, 0.05, 20)

rospy.spin()
