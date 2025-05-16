#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # Create a new publisher to publish the filtered data to /odom
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    odom_pub.publish(msg)

def listener():
    rospy.init_node('filtered_odom_republisher', anonymous=True)
    rospy.Subscriber("/odometry/filtered", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

