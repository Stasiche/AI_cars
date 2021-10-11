#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 
from time import sleep
from turtlesim.msg import Pose 
import numpy as np
from numpy.linalg import norm

class Turtle:
    def __init__(self):
        self.pos = None
        self.n = None
    
    def update_pos(self, p: Pose):
        self.pos = np.array([float(p.x), float(p.y)])
        self.theta = float(p.theta)
        self.n = np.array([np.cos(p.theta), np.sin(p.theta)])

    def calc_pos_diff(self, other: 'Turtle') -> float:
        diff = self.pos - other.pos
        return diff



def calc_move(diff: np.ndarray, turtle: 'Turtle') -> Twist:
    result = Twist()

    norm_diff = norm(diff)
    diff_ = diff/norm_diff if norm_diff != 0 else np.array([0,0])

    result.linear.x = norm_diff if norm_diff < 10 else np.log(norm_diff)
    
    turtle_angle = np.arccos(diff_ @ turtle.n)
    result.angular.z = turtle_angle * np.sign(np.cross(turtle.n, diff_))

    return result

def callback(msg):
    turtle1.update_pos(msg)

    diff = turtle1.calc_pos_diff(leo)
    leo_twist = calc_move(diff, leo)
    pub.publish(leo_twist)

rospy.init_node('turtle_1_follower')

turtle1 = Turtle()
leo = Turtle()

pub = rospy.Publisher('/leo/cmd_vel', Twist, queue_size=1)
rospy.Subscriber('/leo/pose', Pose, leo.update_pos)
sleep(0.5)
rospy.Subscriber('/turtle1/pose', Pose, callback)
rospy.spin()
