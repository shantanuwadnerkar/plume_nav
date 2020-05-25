#! /usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math
from tf.transformations import euler_from_quaternion
import sys
import math
#import actionlib
#from move_robot.msg import waypointAction, waypointGoal, waypointResult, waypointFeedback


x = 0; y =0; theta = 0
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

    # direction = (a-b)/abs(a-b)
    normdeg = (a-b)%(2*math.pi)
    diff = min(2*math.pi-normdeg, normdeg)
    return diff

def moving_controller(goal):
        if abs(goal - theta) > 0.01:
            vel_msg.angular.z = 2 * angdiff(theta, goal)
            rospy.loginfo("angdiff %f"%angdiff(theta,goal))
            return False
        else:
            vel_msg.angular.z = 0
            return True

        

def h():
    print("[COMPLETED] Reached. Shutting down")


def do_move():
    rospy.init_node("rotating_dude",disable_signals=True)
    rospy.on_shutdown(h)

    sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, callbackfn)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    done = False
    rospy.loginfo("[x = %f, y = %f], theta = %f", x,y, theta)
    #rospy.wait_for_message("/cmd_vel", Twist)
    r = rospy.Rate(10)
    i = 0

    goal = float(sys.argv[1]) * math.pi/180
    rospy.loginfo("Goal %f"%goal)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        if not done:
            done = moving_controller(goal)
        if done:
            i += 1
            if i == 3:
                rospy.signal_shutdown("Reached. Shutting down")
        pub.publish(vel_msg)
        rospy.loginfo("[x = %f, y = %f], theta = %f", x,y, theta)
        r.sleep()

# def moving_callback():



# def mover_action():
#     rospy.init_node("action_server")
#     server = actionlib.SimpleActionServer('waypoints', waypointAction, moving_callback, False)
#     server.start()
#     rospy.spin()



if __name__ == "__main__":
    if len(sys.argv) == 2:
        do_move()
    else:
        print('error2') # mover_action()
else:
    print("error")
