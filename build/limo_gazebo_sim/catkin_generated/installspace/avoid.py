#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from math import radians, degrees, atan2

class LineAndObstacleNavigator:
    def __init__(self):
        rospy.init_node('line_and_obstacle_navigator', anonymous=True)

        # ROS Subscribers and Publishers
        self.image_sub = rospy.Subscriber('/limo/rgb/image_raw', Image, self.image_callback)
        self.imu_sub = rospy.Subscriber('/limo/imu', Imu, self.imu_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # OpenCV and Twist objects
        self.bridge = CvBridge()
        self.twist = Twist()

        # HSV Color Ranges for detection
        self.orange_lower = np.array([5, 100, 100])
        self.orange_upper = np.array([15, 255, 255])
        self.blue_lower = np.array([100, 150, 50])
        self.blue_upper = np.array([130, 255, 255])

        self.red_lower = np.array([0, 150, 100])
        self.red_upper = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 150, 100])
        self.red_upper2 = np.array([180, 255, 255])

        # State Variables
        self.detecting_orange = True
        self.turning = False
        self.current_yaw = None
        self.initial_yaw = None
        self.target_yaw = None
        self.red_detected_flag = False

        # PID Parameters for Angular Control
        self.kp = 0.03
        self.ki = 0.0005
        self.kd = 0.01
        self.previous_error = 0
        self.integral = 0

        self.linear_speed = 0.2
        self.angular_speed = 0.2

        rospy.sleep(1)
        self.set_initial_yaw()

    def imu_callback(self, msg):
        # Update current yaw from IMU
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy ** 2 + qz ** 2)
        self.current_yaw = degrees(atan2(siny_cosp, cosy_cosp))
        if self.current_yaw < 0:
            self.current_yaw += 360

    def set_initial_yaw(self):
        """Set the initial yaw for the robot."""
        rospy.loginfo("Setting initial yaw...")
        while self.current_yaw is None:
            rospy.sleep(0.1)
        self.initial_yaw = self.current_yaw
        rospy.loginfo(f"Initial Yaw set to: {self.initial_yaw}°")
        self.move_straight()

    def move_straight(self):
        """Move straight at a constant speed."""
        rospy.loginfo("Moving straight...")
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def steer_left_function(self):
        """Steer left if red object detected."""
        rospy.loginfo("Red object detected: Executing steering function...")
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(3)
        
        # Steer left
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(6)

        # Move straight
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)

        # Steer right back to path
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = -self.angular_speed
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(10)
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)
        
        # Resume moving straight
        self.move_straight()

    def image_callback(self, msg):
        """Process the camera image and detect both lines and red objects."""
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define ROI (adjust for your image size)
        height, width, _ = cv_image.shape
        roi = hsv_image[int(height / 2):height, 0:width]

        # Mask for colors
        orange_mask = cv2.inRange(roi, self.orange_lower, self.orange_upper)
        blue_mask = cv2.inRange(roi, self.blue_lower, self.blue_upper)
        red_mask1 = cv2.inRange(roi, self.red_lower, self.red_upper)
        red_mask2 = cv2.inRange(roi, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Detection Flags
        orange_detected = cv2.countNonZero(orange_mask) > 500
        blue_detected = cv2.countNonZero(blue_mask) > 500
        red_detected = cv2.countNonZero(red_mask) > 200

        # Handle red object detection
        if red_detected and not self.red_detected_flag:
            self.red_detected_flag = True
            rospy.loginfo("Red object detected, starting movement!")
            self.steer_left_function()

        elif not red_detected:
            self.red_detected_flag = False

        # Handle line detection
        if not self.turning:
            if orange_detected:
                rospy.loginfo("Orange detected: Moving straight...")
                self.steer_left_function()

            elif blue_detected:
                rospy.loginfo("Blue detected: Initiating turn...")
                self.start_turn()

        # Optional: Visualization for debugging
        cv2.imshow("Orange Mask", orange_mask)
        cv2.imshow("Blue Mask", blue_mask)
        cv2.imshow("Red Mask", red_mask)
        cv2.waitKey(1)
  
     
  
  
    def start_turn(self):
        """Start turn logic."""
        self.target_yaw = (self.initial_yaw + 90) % 360
        self.turning = True
        rospy.loginfo(f"Target Yaw: {self.target_yaw}°")
        
    def execute_turn(self):
        """Perform precise turn with PID control."""
        if self.current_yaw is None or self.target_yaw is None:
            return

        error = self.target_yaw - self.current_yaw
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        self.integral += error
        derivative = error - self.previous_error
        angular_z = self.kp * error + self.ki * self.integral + self.kd * derivative
        angular_z = np.clip(angular_z, -0.4, 0.4)

        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(self.twist)

        if abs(error) < 1.0:
            rospy.loginfo(f"Turn completed. Current Yaw: {self.current_yaw}°")
            self.turning = False
            self.initial_yaw = self.current_yaw
            self.move_straight()

        self.previous_error = error

    def run(self):
        rospy.loginfo("Line and Obstacle Navigator started...")
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = LineAndObstacleNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass

