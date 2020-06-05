#! /usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math
from tf.transformations import euler_from_quaternion
import sys
#import actionlib
#from move_robot.msg import waypointAction, waypointGoal, waypointResult, waypointFeedback


x = []; y = []; theta = []
vel_msg = Twist()

def callbackfn(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch,theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def angdiff(a, b):

    direction = (a-b)/abs(a-b)
    normdeg = (a-b)%(2*math.pi)
    diff = min(2*math.pi-normdeg, normdeg)
    return diff

def moving_controller(goal_x, goal_y):
        dist2goal = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
        if dist2goal > 0.05:
            angle_to_goal = math.atan2((goal_y - y), (goal_x - x))
            if abs(angle_to_goal - theta) > 0.01:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 3 * angdiff(theta, angle_to_goal)
            else:
                vel_msg.linear.x = min(0.5,2 * dist2goal)
                vel_msg.angular.z = 0
            return False
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            return True
        

def h():
    print("[COMPLETED] Reached. Shutting down")


def do_move():
    rospy.init_node("moving_dude",disable_signals=True)
    rospy.on_shutdown(h)

    sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, callbackfn)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    done = False
    
    rospy.wait_for_message("/base_pose_ground_truth", Odometry)
    r = rospy.Rate(10)
    i = 0

    goal = Point()
    goal.x = float(sys.argv[1])
    goal.y = float(sys.argv[2])

    while not rospy.is_shutdown():
        if not done:
            done = moving_controller(goal.x, goal.y)
        if done:
            i += 1
            if i == 3:
                rospy.signal_shutdown("Reached. Shutting down")
        pub.publish(vel_msg)
        rospy.loginfo("[%f,%f], %f", x,y, theta)
        r.sleep()

# def moving_callback():



# def mover_action():
#     rospy.init_node("action_server")
#     server = actionlib.SimpleActionServer('waypoints', waypointAction, moving_callback, False)
#     server.start()
#     rospy.spin()

# print("NAME:======= ",__name__)

# if __name__ == "__main__":
#     if len(sys.argv) == 3:
#         do_move()
#         print('123123')
#     else:
#         print('error2') # mover_action()
# else:
#     print("error")

do_move()