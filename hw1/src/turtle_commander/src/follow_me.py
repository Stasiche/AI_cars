#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 
import time
from turtlesim.msg import Pose 
import numpy as np
from numpy.linalg import norm

class Turtle:
    def __init__(self, x0: float = 0, y0: float = 0):
        self.pos = np.array([x0, y0])
        self.theta = 0
    
    def update_pos(self, p: Pose):
        self.pos = np.array([float(p.x), float(p.y)])

    def update_theta(self, p: Pose):
        self.theta = float(p.theta)

    def update_pos_by_twist(self, t: Twist):
        self.pos[0] += t.linear.x
        self.pos[1] += t.linear.y

    def update_theta_by_twist(self, t: Twist):
        self.theta += t.angular.z
    
    def calc_pos_diff(self, other: 'Turtle') -> float:
        return self.pos - other.pos



def calc_move(diff) -> Twist:
    result = Twist()
    result.linear.x = diff[0]
    result.linear.y = diff[1]
    
    turtle_angle = np.arccos(diff[0] / norm(diff)) if norm(diff) != 0 else 0
    result.angular.z = turtle_angle - leo.theta 

    return result

def callback(msg):
    if str(msg.x) != 'nan':
        turtle1.update_pos(msg)

        diff = turtle1.calc_pos_diff(leo)
        leo_twist = calc_move(diff)
        pub.publish(leo_twist)


rospy.init_node('turtle_1_follower')

turtle1 = Turtle()
leo = Turtle()

pub = rospy.Publisher('/leo/cmd_vel', Twist, queue_size=1)
rospy.Subscriber('/turtle1/pose', Pose, callback)
rospy.Subscriber('/leo/pose', Pose, leo.update_pos)
rospy.Subscriber('/leo/pose', Pose, leo.update_theta)

rospy.spin()
# msg = Twist()
# msg.linear.x = 1.0
# msg.angular.z = 1.0
# r = rospy.Rate(1)
# while not rospy.is_shutdown():
#     pub.publish(msg)
#     r.sleep()
