#!/usr/bin/env python

import math
import sys

import rospy

import actionlib
from crazyflie_control.msg import waypointGoal, waypointAction, waypointResult, waypointFeedback
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from olfaction_msgs.msg import anemometer, gas_sensor
import time
from tf.transformations import euler_from_quaternion

class RasterScan:
    def __init__(self):
        self.rviz_time = 0.0
        self.wind_speed = 0.0
        self.wind_direction = 0.0
        self.initial_average_wind = True
        self.initial_average_wind_count = 0
        self.initial_average_wind_direction = 0.0



    def rasterScan(self):
        pass


    def wind_callback(self, msg):
        if self.initial_average_wind:
            self.initial_average_wind_direction += msg.wind_direction
        # self.wind_speed = msg.wind_speed
        # self.wind_direction = msg.wind_direction


if __name__=="__main__":
    rospy.init_node("raster_scan")
    rs = RasterScan()
    rospy.Subscriber("Anemometer/WindSensor_reading", anemometer, callback=rs.wind_callback)