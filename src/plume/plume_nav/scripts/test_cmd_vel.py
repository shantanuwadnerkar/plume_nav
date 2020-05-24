#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

rospy.init_node("velocity")
publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

rate = rospy.Rate(100)

velocity = Twist()
velocity.linear.x = 0.0
velocity.linear.y = 0.0
velocity.linear.z = 1.0
velocity.angular.x = 0.0
velocity.angular.y = 0.0
velocity.angular.z = 0.0

i = 0.0

while not rospy.is_shutdown():
    velocity.linear.x = 2.0
    
    publisher.publish(velocity)

    i+= 0.01
    rate.sleep()