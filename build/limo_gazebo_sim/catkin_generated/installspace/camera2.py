#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import math

# Global Variables
yaw_angle = 0.0  # IMU yaw angle
blue_detected = False
orange_detected = False
lines_passed = 0  # Tracks if we passed one line or two
turning = False
target_yaw = 0.0

# ROS Topics
CMD_VEL_TOPIC = '/cmd_vel'
IMU_TOPIC = '/limo/imu'
CAMERA_TOPIC = '/limo/rgb/image_raw'

# Initialize ROS Node and Publishers
rospy.init_node('wro_robot_controller')
cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
bridge = CvBridge()

# Function to Process IMU Data for Drift Correction
def imu_callback(data):
    global yaw_angle
    # Extract the yaw (z-axis rotation) from IMU quaternion
    quaternion = data.orientation
    siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2)
    yaw_angle = math.atan2(siny_cosp, cosy_cosp)
    rospy.loginfo(f"Current Yaw Angle: {math.degrees(yaw_angle)}")

# Function to Detect Colors (Blue and Orange)
def image_callback(msg):
    global blue_detected, orange_detected, lines_passed
    try:
        # Convert ROS Image message to OpenCV format
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        # Define HSV range for blue
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Define HSV range for orange
        lower_orange = np.array([5, 150, 50])
        upper_orange = np.array([15, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # Check for orange and blue detection
        if cv2.countNonZero(mask_orange) > 500 and lines_passed == 0:  # Threshold for detection
            orange_detected = True
            rospy.loginfo("Orange Line Detected!")
            lines_passed = 1
        if cv2.countNonZero(mask_blue) > 500 and lines_passed == 1:
            blue_detected = True
            rospy.loginfo("Blue Line Detected!")
            lines_passed = 2

        # Display for debugging
        combined_mask = cv2.bitwise_or(mask_blue, mask_orange)
        cv2.imshow("Color Detection", combined_mask)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Error in image processing: {e}")

# Function to Move the Robot

def move_robot(linear=0.0, angular=0.0):
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    cmd_pub.publish(twist)

# Main Function
if __name__ == '__main__':
    global turning, target_yaw, yaw_angle, lines_passed, blue_detected, orange_detected #<--- MOVED HERE!
    try:
        # Subscribers
        rospy.Subscriber(IMU_TOPIC, Imu, imu_callback)
        rospy.Subscriber(CAMERA_TOPIC, Image, image_callback)

        rospy.loginfo("Starting WRO Robot Controller")
        rate = rospy.Rate(10)  # 10 Hz


        while not rospy.is_shutdown():
            # 1. Move Straight Initially
            if lines_passed < 2 and not turning:
                # Correct drift based on yaw angle feedback
                drift_correction = -yaw_angle  # Opposite of yaw to counteract drift
                move_robot(linear=0.2, angular=drift_correction)

            # 2. Once Both Lines Are Passed, Make a Turn
            elif lines_passed == 2 and not turning:
                rospy.loginfo("Both Lines Passed - Preparing to Turn")
                target_yaw = yaw_angle + math.pi/2 # Set target angle
                turning = True
                lines_passed = 0
                blue_detected = False
                orange_detected = False

            # Perform rotation until target is reached
            elif turning:
                current_yaw = yaw_angle
                angle_diff = target_yaw - current_yaw

                # Normalize the angle difference to be between -pi and pi
                if angle_diff > math.pi:
                   angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                # Set Angular Speed Based on Angle Difference
                if abs(angle_diff) > 0.1:
                    turn_speed = 0.5 if angle_diff > 0 else -0.5
                    move_robot(linear=0.0, angular=turn_speed)
                else:
                  move_robot(linear = 0, angular = 0)
                  turning = False
                  yaw_angle = 0.0

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("WRO Robot Controller Stopped")
    finally:
        cv2.destroyAllWindows()
