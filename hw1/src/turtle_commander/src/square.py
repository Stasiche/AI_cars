#!/usr/bin/env python

import rospy
import time
import numpy as np

from geometry_msgs.msg import Twist 
# from geometry_msgs.msg import Pose, Point, Quaternion
from turtlesim.msg import Pose
import matplotlib.pyplot as plt


# points = []
# def callback(msg):
#     points.append(float(msg.x))

# rospy.init_node('turtle_square')
# pub = rospy.Publisher('/turtle1/pose', Pose, queue_size=100)
# rospy.Subscriber('/turtle1/pose', Pose, callback)

# msg = Pose()
# r = rospy.Rate(200)
# k = int(1e2)
# while not rospy.is_shutdown() and k:
#     msg.x = np.random.random()
#     pub.publish(msg)
#     r.sleep()
#     k -= 1

# print(len(points))
# plt.plot(points)
# plt.show()

rospy.init_node('turtle_square')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
msg = Twist()
msg.linear.x = -3.0
msg.angular.z = 1.0

r = rospy.Rate(1)
while not rospy.is_shutdown():
    pub.publish(msg)
    r.sleep()
