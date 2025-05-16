#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from math import atan2, degrees

class LimoController:
    def __init__(self):
        rospy.init_node("limo_controller", anonymous=True)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/limo/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/limo/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber('/limo/imu', Imu, self.imu_callback)

        # Control parameters
        self.bridge = CvBridge()
        self.twist = Twist()
        self.lidar_data = None
        self.min_distance = float("inf")  # Initialize to a large value
        self.detected_color = None
        self.turning = False
        self.current_yaw = None
        self.initial_yaw = None
        self.target_yaw = None

        # PID Parameters for Angular Control
        self.kp = 0.03
        self.ki = 0.0005
        self.kd = 0.01
        self.previous_error = 0
        self.integral = 0
        self.yaw_margin = 1.0

        # HSV color ranges
        self.red_lower1 = np.array([0, 150, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 150, 100])
        self.red_upper2 = np.array([180, 255, 255])
        self.green_lower = np.array([40, 100, 50])
        self.green_upper = np.array([80, 255, 255])
        self.orange_lower = np.array([5, 100, 100])
        self.orange_upper = np.array([15, 255, 255])
        self.blue_lower = np.array([100, 150, 50])
        self.blue_upper = np.array([130, 255, 255])

        self.orange_detected_flag = False
        self.blue_detected_flag = False
        self.has_moved = False  # Add this flag
        

        # State management
        self.state = "IDLE"  # Possible states: IDLE, OBJECT_TASK, LINE_TASK
        self.last_object_detection_time = rospy.Time.now()  # Cooldown
        self.object_detection_cooldown = 0.2  # Cooldown duration in seconds
        rospy.loginfo("Limo Controller Node Initialized")

        # Start moving straight
        self.set_initial_yaw()

        rospy.loginfo("Starting robot movement.")

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
          # Start moving forward

    def lidar_callback(self, msg):
        """Process LiDAR data to find the closest object."""
        self.lidar_data = msg  # Store full LiDAR message for forward check
        valid_ranges = [r for r in msg.ranges if 0.02 < r < 2.0]  # Valid range: 2 cm to 2 meters
        self.min_distance = min(valid_ranges) if valid_ranges else float('inf')
        rospy.loginfo(f"Minimum LiDAR distance: {self.min_distance:.2f} meters")
        

    def image_callback(self, msg):
        """Handle RGB image and make decisions."""
        self.move_straight()
        try:
            if self.state == "IDLE" and self.min_distance < 2.0 and (rospy.Time.now() - self.last_object_detection_time) > rospy.Duration(self.object_detection_cooldown):
                self.detect_object(msg)
            elif self.state in ["IDLE", "LINE_TASK"]:
                self.detect_line(msg)

            if self.state != "OBJECT_TASK":
                self.has_moved = False
        except Exception as e:
            rospy.logerr(f"Image callback error: {e}")

    def detect_object(self, msg):
        """Detect objects based on color in the image and handle tasks."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define ROIs for object detection
        object_roi = hsv_image[int(cv_image.shape[0] * 0.25):int(cv_image.shape[0] * 0.75), :]

        # Create masks for colors
        red_mask = cv2.inRange(object_roi, self.red_lower1, self.red_upper1) + cv2.inRange(object_roi, self.red_lower2, self.red_upper2)
        green_mask = cv2.inRange(object_roi, self.green_lower, self.green_upper)

        if np.count_nonzero(red_mask) > 500:
            rospy.loginfo("Red object detected within range.")
            self.state = "OBJECT_TASK"
            self.steer_left()
            cv2.imshow("red Mask", red_mask)
            rospy.sleep(2.0)
            self.check_and_continue()
            self.last_object_detection_time = rospy.Time.now()
        elif np.count_nonzero(green_mask) > 500:
            rospy.loginfo("Green object detected within range.")
            self.state = "OBJECT_TASK"
            self.steer_right()
            cv2.imshow("green Mask", green_mask)
            rospy.sleep(2.0)
            self.check_and_continue()
            self.last_object_detection_time = rospy.Time.now()

    def detect_line(self, msg):
        """Detect lines based on color in the image and handle accordingly."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        height, width, _ = cv_image.shape
        roi = hsv_image[400:500, 0:width]

        # Create masks for line colors
        orange_mask = cv2.inRange(roi, self.orange_lower, self.orange_upper)
        blue_mask = cv2.inRange(roi, self.blue_lower, self.blue_upper)

        orange_detected = cv2.countNonZero(orange_mask) > 500
        blue_detected = cv2.countNonZero(blue_mask) > 500

        if not self.turning:
            if orange_detected and not self.orange_detected_flag:
                rospy.loginfo("Orange line detected: Steering smoothly.")
                self.steer_smoothly()
                cv2.imshow("Orange Mask", orange_mask)
                self.state = "LINE_TASK"
                self.orange_detected_flag = True
                rospy.sleep(0.5)
            elif not orange_detected:
                self.orange_detected_flag = False

            if blue_detected and not self.blue_detected_flag:
                rospy.loginfo("Blue line detected: Starting turn.")
                self.start_turn()
                cv2.imshow("Blue Mask", blue_mask)
                self.state = "LINE_TASK"
                self.blue_detected_flag = True
                rospy.sleep(0.5)
            elif not blue_detected:
                self.blue_detected_flag = False
        elif self.turning:
            self.execute_turn()

    def check_and_continue(self):
        """Check if it's safe to move straight after steering."""
        rospy.loginfo("Checking if forward path is clear...")
        rospy.sleep(1)

        if self.lidar_data is not None:
            forward_ranges = []
            angle_range = 20
            lidar_size = len(self.lidar_data.ranges)
            center_index = lidar_size // 2

            for i in range(center_index - angle_range, center_index + angle_range + 1):
                if 0 <= i < lidar_size and 0.02 < self.lidar_data.ranges[i] < 2.0:
                    forward_ranges.append(self.lidar_data.ranges[i])

            forward_min_distance = min(forward_ranges) if forward_ranges else float('inf')

            if forward_min_distance > 0.3 and not self.has_moved:
                rospy.loginfo("Forward path is clear. Continuing straight...")
                self.move_straight()
                self.state = "IDLE"
                self.has_moved = True
            else:
                rospy.logwarn(f"Obstacle detected: {forward_min_distance:.2f} meters. Stopping!")
                self.stop()

    def steer_left(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.3
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)

    def steer_right(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = -0.3
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(3)

    def move_straight(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def steer_smoothly(self):
        rospy.loginfo("Performing smooth steering...")
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.3
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)

    def start_turn(self):
        if self.current_yaw is not None:
            self.target_yaw = (self.initial_yaw + 90) % 360
            self.turning = True
            self.previous_error = 0
            self.integral = 0
            rospy.loginfo(f"Target Yaw: {self.target_yaw}°")

    def execute_turn(self):
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

        self.twist.linear.x = 0.1
        self.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(self.twist)

        if abs(error) < self.yaw_margin:
            rospy.loginfo(f"Turn completed. Current Yaw: {self.current_yaw}°")
            self.turning = False
            self.initial_yaw = self.current_yaw
            self.move_straight()

        self.previous_error = error

    def stop(self):
        rospy.loginfo("Stopping the robot.")
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)
        self.twist.linear.x = -0.1
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

if __name__ == "__main__":
    try:
        LimoController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

