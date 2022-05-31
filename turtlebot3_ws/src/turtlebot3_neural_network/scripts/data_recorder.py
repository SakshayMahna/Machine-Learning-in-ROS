#!/usr/bin/env python
import rospy
import csv
import message_filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

filename = "data.csv"
csvfile = open(filename, "w")
csvwriter = csv.writer(csvfile)
dataset = []

def data_callback(laser, twist):
    laser_data = laser.ranges[:60] + laser.ranges[-60:]
    twist_data = (twist.linear.x, twist.angular.z)
    data = laser_data + twist_data
    dataset.append(data)

if __name__ == "__main__":
    scan_sub = message_filters.Subscriber('scan', LaserScan, queue_size = 1)
    cmd_sub = message_filters.Subscriber('cmd_vel', Twist, queue_size = 1)

    rospy.init_node('data_recorder')

    ts = message_filters.ApproximateTimeSynchronizer([scan_sub, cmd_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(data_callback)

    counter = 0
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        rate.sleep()
        counter += 1 

        if counter > 1e5:
            break

    csvwriter.writerows(dataset)