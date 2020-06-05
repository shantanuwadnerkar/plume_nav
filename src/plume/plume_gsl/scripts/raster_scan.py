#!/usr/bin/env python

import math
import time
import sys

import rospy

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from olfaction_msgs.msg import anemometer, gas_sensor
import tf
from tf.transformations import euler_from_quaternion

from move_drone_client import MoveDroneClient


class RasterScan:
    def __init__(self, scan_distance):
        self.move_drone_client = MoveDroneClient()

        self.wind_speed = 0.0
        self.wind_direction = 0.0

        self.avg_wind_direction_duration = 2
        self.avg_wind = True
        self.avg_wind_first_time = True

        self.rs_start_position = self.move_drone_client.get_drone_position()
        self.rs_scan_distance = scan_distance
        self.rs_scanned_distance = 0.0

        self.wind_velocity_x = 0.0
        self.wind_velocity_y = 0.0
        self.avg_wind_velocity_x = 0.0
        self.avg_wind_velocity_y = 0.0
        self.avg_wind_count = 0
        self.avg_wind_direction_start_time = rospy.Time.now().secs
        
        self.transform_listener = tf.TransformListener()
        self.fixed_frame  = rospy.get_param("/fixed_frame")
        self.anemometer_frame  = rospy.get_param("/anemometer_frame")
        rospy.Subscriber("Anemometer/WindSensor_reading", anemometer, callback=self.wind_callback)
        rospy.wait_for_message("Anemometer/WindSensor_reading", anemometer)


    def startRasterScan(self):
        print("startRasterScan")
        if self.avg_wind_first_time:
            sleep_duration = rospy.Duration(secs=self.avg_wind_direction_duration)
            rospy.sleep(sleep_duration)
            self.avg_wind_first_time = False

        self.rs_start_position = self.move_drone_client.get_drone_position()

        heading = self.getPerpendicularAngle(self.avg_wind_direction)
        
        self.flankScan(heading)
        self.rs_scanned_distance = 0
        heading = self.getOppositeAngle(heading)
        print(self.rs_start_position)
        self.move_drone_client.sendWaypoint(self.rs_start_position)
        self.flankScan(heading)

    def flankScan(self, heading):
        print("rasterScan")

        while self.rs_scanned_distance <= self.rs_scan_distance and not rospy.is_shutdown():
            # if concentration goes below a certain level, go back
            # if False:
            #     break

            waypoint = self.move_drone_client.generateWaypoint(heading)
            self.move_drone_client.sendWaypoint(waypoint)
            self.rs_scanned_distance = self.findScannedDistance(self.move_drone_client.get_drone_position())

        self.rs_scanned_distance = 0.0


    def findScannedDistance(self, current_position):
        return math.sqrt((current_position[0] - self.rs_start_position[0])**2 + (current_position[1] - self.rs_start_position[1])**2 + (current_position[2] - self.rs_start_position[2])**2)


    def wind_callback(self, msg):
        self.wind_speed = msg.wind_speed
        self.wind_direction = self.anemometer_transform(msg.wind_direction)

        self.wind_velocity_x = msg.wind_speed * math.cos(self.wind_direction)
        self.wind_velocity_y = msg.wind_speed * math.sin(self.wind_direction)

        if self.avg_wind:
            if (rospy.Time.now().secs - self.avg_wind_direction_start_time < self.avg_wind_direction_duration):
                self.avg_wind_count += 1
                self.avg_wind_velocity_x  = self.cumulative_moving_average(self.wind_velocity_x, self.avg_wind_count, self.avg_wind_velocity_x)
                self.avg_wind_velocity_y  = self.cumulative_moving_average(self.wind_velocity_y, self.avg_wind_count, self.avg_wind_velocity_y)    
            else:
                self.avg_wind = False
                self.avg_wind_direction = math.atan2(self.avg_wind_velocity_y, self.avg_wind_velocity_x)
                print(self.avg_wind_direction)


    def cumulative_moving_average(self, new_val, num_val, prev_avg):
        avg = prev_avg + (new_val - prev_avg) / (num_val + 1)
        return avg


    def anemometer_transform(self, wind_direction_msg):
        self.transform_listener.waitForTransform(self.anemometer_frame,self.fixed_frame, rospy.Time(), rospy.Duration(5.0))

        # Orientation of sensor with respect to world frame
        _,rot = self.transform_listener.lookupTransform(self.fixed_frame, self.anemometer_frame, rospy.Time(0))
        _, _, yaw = euler_from_quaternion(rot)
        wind_direction = yaw + wind_direction_msg - math.pi

        # Normalizing angle
        return self.normalize_angle(wind_direction)


    def getPerpendicularAngle(self, angle):
        return self.normalize_angle(angle - math.pi/2)


    def getOppositeAngle(self, angle):
        return self.normalize_angle(angle - math.pi)


    def normalize_angle(self, angle):
        # if angle <= -math.pi:
        #     return angle % -math.pi

        # if angle > math.pi:
        #     return angle % math.pi
        while angle <= -math.pi:
            angle += 2*math.pi

        while angle > math.pi:
            angle -= 2*math.pi

        return angle



if __name__=="__main__":
    rospy.init_node("raster_scan")

    rs = RasterScan(0.4)
    rs.startRasterScan()

    rospy.spin()