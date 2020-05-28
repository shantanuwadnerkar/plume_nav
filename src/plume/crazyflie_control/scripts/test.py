#!/usr/bin/env python
import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from olfaction_msgs.msg import anemometer, gas_sensor
from tf.transformations import euler_from_quaternion
import sys
import math
import csv



# Global variables 
windspeed = 0.0
wind_dir = 0.0
conc = 0.0
x = 0.0; y =0.0; theta = 0.0 #change this to x = [] and see if you can remove sleep
vel_msg = Twist()
check = 0



# Function (3) to get the wind data
def wind_callbck(msg):
    global windspeed
    global wind_dir
    windspeed = msg.wind_speed
    wind_dir = msg.wind_direction
    
    #rospy.loginfo("wind speed: %s" , windspeed)
    #rospy.loginfo("wind direction: %s", wind_dir)

# Function (4) to get the gas concentration, 
def conc_callbck(msg):
    global conc
    conc = msg.raw
    #rospy.loginfo("conc: %f",conc)

# Function (5) to get the current location and speed 
def loc_callbck(msg):

    global x 
    global y
    global theta 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch,theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w]) # not sure how this works, copied from mover.py
    #rospy.loginfo("x: %f",x)
    #rospy.loginfo("y: %f",y)
    #rospy.loginfo("Theta: %f",theta)

# Function (6) to get velocity co-ordinates (used in conjuction with the function below)
def moving_controller(goal_x, goal_y):
        dist2goal = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
        if dist2goal > 0.05:
            angle_to_goal = math.atan2((goal_y - y), (goal_x - x))
            if abs(angle_to_goal - theta) > 0.05:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 4 * angdiff(theta, angle_to_goal)
            else:
                vel_msg.linear.x = dist2goal
                vel_msg.angular.z = 0
            return False
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            return True
        

# Function to announce completion of movement, called in do_move()
def h():
    print("[COMPLETED] Reached. Shutting down")

def angdiff(a, b):

    direction = (a-b)/abs(a-b)
    normdeg = (a-b)%(2*math.pi)
    diff = min(2*math.pi-normdeg, normdeg)
    return diff

# Function (7) to move the robot to desired co-ordinates,calls the function above (moving_controller)
def do_move(a,b):
    #rospy.on_shutdown(h)
    done = False
    
    #rospy.wait_for_message("/cmd_vel", Twist)
    r = rospy.Rate(10)
    i = 0
    goal = Point()
    goal.x = a
    goal.y = b

    while not rospy.is_shutdown():
        if not done:
            done = moving_controller(goal.x, goal.y)
        if done:
            i += 1
            if i == 3:
                print("Reached. Shutting down")
                break
        pub_speed.publish(vel_msg)
        rospy.loginfo("[%f,%f], %f", x,y, theta)
        r.sleep()


# Function to store the data   
def testing(): 
    with open('/home/adithya/plume_nav/src/plume/crazyflie_control/scripts/writeData.csv','w') as file:
        csvwriter = csv.writer(file, dialect='excel', delimiter=',')
        fields = ['x' , 'y', 'wind speed', 'wind direction ', 'concentration']
        csvwriter.writerow(fields)
        do_move(1,1)
        global check
        if(check == 0):
                rospy.sleep(1) # had to sleep, otherwise x1 and y1 were input as 0
                check = check +1  
        for i in range (1,19):
            for j in range (1,19): 
                do_move(i,j)
                data = [x,y,windspeed,wind_dir,conc]
                csvwriter.writerow(data)
                print("Printed",i,j)
    print("Done")

        
# Function to identify the direction of increasing concentration 
#def which_way():
#    if conc 0: 


    

#if __name__ == "__main__":
#    read_topics()
#    print("--------------------------------")
#    time.sleep(5)
#else: 
#    print("ERROR ######################")


# Function (1) to subscribe to all required topics wind, PID, current location and speed 

rospy.init_node("grad", anonymous=True) # initialize the node 
rospy.Subscriber("Anemometer/WindSensor_reading", anemometer, wind_callbck) # Subscribe to Anemometer - get wind data
rospy.Subscriber("PID/Sensor_reading", gas_sensor, conc_callbck) # Subscribe to PID data 
rospy.Subscriber("/base_pose_ground_truth", Odometry, loc_callbck) # Current position and speed 

# Function (2) to publish to nodes - cmd_vel
pub_speed = rospy.Publisher("/cmd_vel", Twist, queue_size=10)



if __name__ == "__main__":
    testing()
else: 
    print("Error")


