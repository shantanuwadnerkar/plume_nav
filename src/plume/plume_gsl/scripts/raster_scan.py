#!/usr/bin/env python

import math
import time
import sys

import rospy

import actionlib
from plume_gsl.msg import rasterScanAction, rasterScanGoal, rasterScanResult, rasterScanFeedback
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from olfaction_msgs.msg import anemometer, gas_sensor
import tf
from tf.transformations import euler_from_quaternion

from move_drone_client import MoveDroneClient


class RasterScan:
    def __init__(self, use_actionlib=True):
        self.move_drone_client = MoveDroneClient()

        self.transform_listener = tf.TransformListener()
        self.fixed_frame  = rospy.get_param("/fixed_frame")
        self.anemometer_frame  = rospy.get_param("/anemometer_frame")

        self.avg_wind_direction_duration = 2
        self.avg_wind = True
        self.avg_wind_first_time = True

        self.avg_wind_count = 0
        self.avg_wind_velocity_x = 0.0
        self.avg_wind_velocity_y = 0.0
        self.avg_wind_direction_start_time = rospy.Time.now().secs

        rospy.Subscriber("PID/Sensor_reading", gas_sensor, callback=self.concentration_callback)
        rospy.Subscriber("Anemometer/WindSensor_reading", anemometer, callback=self.anemometer_callback)
        rospy.wait_for_message("PID/Sensor_reading", gas_sensor)
        rospy.wait_for_message("Anemometer/WindSensor_reading", anemometer)

        self.rs_start_position = self.move_drone_client.get_drone_position()
        # self.rs_start_position = self.move_drone_client.position
        self.rs_scan_distance = 0.0
        self.rs_scanned_distance = 0.0
        self.rs_max_concentration_position_tuple = (self.current_concentration, self.move_drone_client.get_drone_position())
        # self.rs_max_concentration_position_tuple = (self.current_concentration, (self.move_drone_client.position.x. \
        #                                             self.move_drone_client.position.y, self.move_drone_client.position.z))        

        if use_actionlib:
            self.action_server = actionlib.SimpleActionServer('rasterScan', rasterScanAction, self.rs_action_callback, False)
            self.action_server.start()
        else:
            pass


    def startRasterScan(self, scan_distance=2.0):
        self.rs_scan_distance = scan_distance

        if self.avg_wind_first_time:
            sleep_duration = rospy.Duration(secs=self.avg_wind_direction_duration)
            rospy.sleep(sleep_duration)
            self.avg_wind_first_time = False

        self.rs_start_position = self.move_drone_client.get_drone_position()
        # self.rs_start_position = self.move_drone_client.position

        heading = self.getPerpendicularAngle(self.avg_wind_direction)

        self.flankScan(heading)
        self.move_drone_client.sendWaypoint(self.rs_start_position)
        self.flankScan(self.getOppositeAngle(heading))
        self.move_drone_client.sendWaypoint(self.rs_max_concentration_position_tuple[1])
        if use_actionlib:
            self.endRasterScan()


    def endRasterScan(self):
        result = rasterScanResult()
        result.max_concentration = self.rs_max_concentration_position_tuple[0]
        result.max_concentration_point = self.rs_max_concentration_position_tuple[1]
        self.action_server.set_succeeded(result)


    def flankScan(self, heading):
        while self.rs_scanned_distance <= self.rs_scan_distance and not rospy.is_shutdown():
            # if concentration goes below a certain level, go back
            # if False:
            #     break

            if self.concentration_position_tuple[0] > self.rs_max_concentration_position_tuple[0]:
                self.rs_max_concentration_position_tuple = self.concentration_position_tuple

            waypoint = self.move_drone_client.generateWaypoint(heading)
            self.move_drone_client.sendWaypoint(waypoint)
            self.rs_scanned_distance = self.findScannedDistance(self.move_drone_client.get_drone_position())
            # self.rs_scanned_distance = self.findScannedDistance(self.move_drone_client.position)

        self.rs_scanned_distance = 0.0


    def getMaximumConcentrationPosition(self):
        return self.rs_max_concentration_position_tuple


    def findScannedDistance(self, current_position):
        return math.sqrt((current_position[0] - self.rs_start_position[0])**2 + (current_position[1] - self.rs_start_position[1])**2 + (current_position[2] - self.rs_start_position[2])**2)
        # return math.sqrt((current_position.x - self.rs_start_position.x)**2 + (current_position.y - self.rs_start_position.y)**2 + (current_position.z - self.rs_start_position.z)**2)


    def concentration_callback(self, concentration_reading):
        self.current_concentration = concentration_reading.raw
        self.concentration_position_tuple = (self.current_concentration, self.move_drone_client.get_drone_position())
        # self.concentration_position_tuple = (self.current_concentration, (self.move_drone_client.position.x. \
        #                                             self.move_drone_client.position.y, self.move_drone_client.position.z))


    def anemometer_callback(self, msg):
        self.wind_speed = msg.wind_speed
        self.wind_direction = self.anemometer_transform(msg.wind_direction)

        self.wind_velocity_x = self.wind_speed * math.cos(self.wind_direction)
        self.wind_velocity_y = self.wind_speed * math.sin(self.wind_direction)

        if self.avg_wind:
            if (rospy.Time.now().secs - self.avg_wind_direction_start_time < self.avg_wind_direction_duration):
                self.avg_wind_count += 1
                self.avg_wind_velocity_x  = self.cumulative_moving_average(self.wind_velocity_x, self.avg_wind_count, self.avg_wind_velocity_x)
                self.avg_wind_velocity_y  = self.cumulative_moving_average(self.wind_velocity_y, self.avg_wind_count, self.avg_wind_velocity_y)    
            else:
                self.avg_wind = False
                self.avg_wind_direction = math.atan2(self.avg_wind_velocity_y, self.avg_wind_velocity_x)


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


    def rs_action_callback(self, goal):
        self.startRasterScan(goal.scan_distance)


if __name__=="__main__":
    rospy.init_node("raster_scan")

    if "--test-wo-actionlib" in sys.argv:
        use_actionlib = False
    else:
        use_actionlib = True

    rs = RasterScan(use_actionlib=use_actionlib)
    
    if not use_actionlib:
        rs.startRasterScan(2)

    rospy.spin()