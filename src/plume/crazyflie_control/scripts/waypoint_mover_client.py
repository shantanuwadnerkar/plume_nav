#! /usr/bin/env python

import rospy
import actionlib
from crazyflie_control.msg import waypointAction, waypointGoal, waypointResult, waypointFeedback
from geometry_msgs.msg import Point

def define_waypoints():
    p = [Point(7,3,0),Point(3,3,0)]
    return p   

def feedback_cb(feedback):
    print('[Feedback] No. of points completed = %d/%d'%(feedback.num_completed,feedback.total_points))


rospy.init_node('action_client')
client = actionlib.SimpleActionClient('waypoints', waypointAction)
client.wait_for_server()

goal = waypointGoal()
goal.points = define_waypoints()
client.send_goal(goal, feedback_cb=feedback_cb)
client.wait_for_result()
print("Completion message: " + client.get_result().succ_msg)