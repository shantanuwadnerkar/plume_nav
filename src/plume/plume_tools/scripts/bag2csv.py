#!/usr/bin/env python

# Script to convert bagfiles to csv
# More topics to be specified


import rosbag
import numpy as np
import os

def readOdom(currentTopic):
    poseList = []
    timeList = []
    for topic, msg, t in currentTopic:
        msgtime = t.to_sec()
        timeList.append(msgtime)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        poseList.append([msgtime, x, y])
    
    return np.array(poseList[1:]), timeList[1:]

def readConcentration(timeList, odom_arr, topics):
    concList = []
    concTime = []

    for topic, msg, t in topics:
        concList.append(msg.data)
        concTime.append(t.to_sec())
    
    concListnorm = []
    lastTime = 0.0
    flag = 0
    
    for i, stdtime in enumerate(timeList):
        data = []
        for j, msgtime in enumerate(concTime):        
            if msgtime <= stdtime:
                data = concList[j]
                flag = 1

        if not data:
            data = 0
        concListnorm.append(data)
        lastTime = stdtime
   
    concListnorm = np.array(concListnorm).reshape(len(concListnorm),1)
    result_arr = np.append(odom_arr,concListnorm,axis=1)

    return result_arr


def convert_files():
 
    dir_name = os.path.dirname(os.path.realpath(__file__))[:-7]+"bagfiles/"
    listofFiles = sorted([f for f in os.listdir(dir_name)])
    try:
        bagFile = listofFiles[-1]
    except IndexError:
        print("No bag files found")
        return
    bag = rosbag.Bag(dir_name+bagFile)

    result_arr, timeList = readOdom(bag.read_messages("/base_pose_ground_truth"))
    result_arr = readConcentration(timeList, result_arr, bag.read_messages("/concentration"))

    headeritems = "time,x,y,concentration"
    np.savetxt(dir_name+"../csvfiles/"+bagFile[:-3]+"csv", result_arr, delimiter=',',header=headeritems,comments='')

    print("Converted to CSV!!")


if __name__ == "__main__":
    convert_files()