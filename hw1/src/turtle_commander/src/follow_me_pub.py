#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 
import time


rospy.init_node('follow_me')

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
msg = Twist()
msg.linear.x = 1.0
msg.angular.z = 1.0
r = rospy.Rate(1)
while not rospy.is_shutdown():
    pub.publish(msg)
    r.sleep()
