#! /usr/bin/python

from nav_msgs.msg import OccupancyGrid, MapMetaData
import rospy
import numpy as np

rospy.init_node('map_node')
pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

r = rospy.Rate(100)
while not rospy.is_shutdown():
    data = 100*np.ones((100,100),dtype=np.int8)
    data[[i for i in range(data.shape[0]) if i%2],:] = np.zeros(data.shape[1])
    grid = OccupancyGrid(data=list(data.ravel()))
    grid.info = MapMetaData()
    grid.info.height = data.shape[0]
    grid.info.width = data.shape[1]
    grid.header.frame_id = 'base_link'
    grid.info.resolution = 0.1

    msg = grid

    pub.publish(msg)
    r.sleep()




