#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
import numpy as np

def pose_to_vec(p: Pose) -> np.ndarray:
    return np.array([float(p.x), float(p.y)])

def callback(msg):
    rospy.loginfo(msg)
    # print(pose_to_vec(msg))


rospy.init_node('turtle_1_pose_consumer')
rospy.Subscriber('/turtle1/pose', Pose, callback)

rospy.spin()
