#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from gridworld import GridWorld
import numpy as np

def find_position(index, m, res):
    a = (index%m)
    b = int(index/m)

    x = a*res
    y = b*res
    
    return x,y

def callbackfn(msg):
    data = np.array(msg.data)
    max_positions = np.where(data == max(data))
    global m        # This can be implemented as class attribute
    global res      # This can be implemented as class attribute
    global max_x
    global max_y
    max_x = 0
    max_y = 0

    for ind in max_positions[0]:
        x, y = find_position(ind, m, res)
        max_x += x
        max_y += y
    max_x /= len(max_positions[0])
    max_y /= len(max_positions[0])
    # return max_x, max_y


rospy.init_node("source_from_map")
grid = GridWorld()
m = grid.m
res = grid.res
max_x = 0
max_y = 0
rospy.Subscriber("mapping_viz",OccupancyGrid, callbackfn)
r = rospy.Rate(2)

while not rospy.is_shutdown():
    rospy.loginfo("max_x = {}, max_y = {}".format(max_x, max_y))
    r.sleep()


# m = 51
# res = 0.4
# class Abcd:
#     def __init__(self):
#         self.data = [0,2600,1300,450]

# msg = Abcd()
# x_max,y_max = callbackfn(msg)
# print("x_max = {}, y_max = {}".format(x_max,y_max))