#! /usr/bin/env python
import rospy
import actionlib
from plume_gsl.msg import rasterScanAction, rasterScanGoal

def done_cb(status,result):
    print("Done")

rospy.init_node("test_client")
client = actionlib.SimpleActionClient('rasterScan', rasterScanAction)
client.wait_for_server()
goal = rasterScanGoal()
goal.scan_distance = 2
client.send_goal(goal, done_cb=done_cb)
rospy.spin()