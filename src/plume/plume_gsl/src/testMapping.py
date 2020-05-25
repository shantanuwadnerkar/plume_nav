#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

m = 51
n = 51

resolution = 0.4

rospy.init_node("testMapping")
pub = rospy.Publisher("mapping_viz", OccupancyGrid, queue_size=10)

prob = OccupancyGrid()
prob.info.height = m
prob.info.width = n
prob.info.resolution = resolution
prob.info.origin.position.x = 0
prob.info.origin.position.y = 0
prob.info.origin.position.z = 0
prob.info.origin.orientation = quaternion_from_euler(0,0,0)
prob.info.map_load_time = rospy.Time.now()
prob.header.frame_id = "map"
alpha   = ((1.0/(m*n)) * np.ones(m*n)).reshape(m*n)
prob.data = list(alpha)

r = rospy.Rate(2)

while not rospy.is_shutdown():

    # prob.info.map_load_time = rospy.Time.now()
    prob.header.stamp = rospy.Time.now()
    pub.publish(prob)
    rospy.loginfo("Published")
    r.sleep()