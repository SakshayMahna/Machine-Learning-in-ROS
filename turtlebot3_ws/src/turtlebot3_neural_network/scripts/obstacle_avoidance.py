#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

g_range_ahead = 1

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges[:60] + msg.ranges[-60:])


if __name__ == "__main__":
    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

    rospy.init_node('obstacle_avoidance')

    state_change_time = rospy.Time.now()
    driving_forward = True
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if driving_forward:
            if g_range_ahead < 0.5:
                driving_forward = False
                state_change_time = rospy.Time.now() + rospy.Duration(random.randrange(3, 6))
        else:
            if rospy.Time.now() > state_change_time:
                driving_forward = True
        
        twist = Twist()
        if driving_forward:
            twist.linear.x = 0.3
        else:
            twist.angular.z = 0.3
        cmd_vel_pub.publish(twist)

        rate.sleep()