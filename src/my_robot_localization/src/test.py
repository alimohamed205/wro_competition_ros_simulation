#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class CornerNavigator:
    def __init__(self):
        rospy.init_node('corner_navigator', anonymous=True)

        # Publishers and Subscribers
        self.image_sub = rospy.Subscriber('/limo/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # OpenCV and Twist
        self.bridge = CvBridge()
        self.twist = Twist()

        # HSV Color Ranges (adjust these if needed)
        self.orange_lower = np.array([5, 100, 100])
        self.orange_upper = np.array([15, 255, 255])
        self.blue_lower = np.array([100, 150, 50])
        self.blue_upper = np.array([130, 255, 255])

        # ROI and Detection States
        self.roi_height = 100
        self.roi_y_offset = 400
        self.detecting_orange = True  # Start with detecting orange
        self.turning = False

        # Initialize the robot to move straight immediately
        rospy.sleep(1)  # Wait to ensure ROS is ready
        self.move_straight()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define ROI
        height, width, _ = cv_image.shape
        roi = hsv_image[self.roi_y_offset:self.roi_y_offset+self.roi_height, 0:width]

        # Color Detection
        orange_mask = cv2.inRange(roi, self.orange_lower, self.orange_upper)
        blue_mask = cv2.inRange(roi, self.blue_lower, self.blue_upper)

        # Morphological operations
        kernel = np.ones((5,5), np.uint8)
        orange_mask = cv2.erode(orange_mask, kernel, iterations=1)
        orange_mask = cv2.dilate(orange_mask, kernel, iterations=1)

        blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
        blue_mask = cv2.dilate(blue_mask, kernel, iterations=1)

        #Thresholding
        _, orange_thresh = cv2.threshold(orange_mask, 1, 255, cv2.THRESH_BINARY)
        _, blue_thresh = cv2.threshold(blue_mask, 1, 255, cv2.THRESH_BINARY)

        orange_detected = cv2.countNonZero(orange_thresh) > 100
        blue_detected = cv2.countNonZero(blue_thresh) > 100


        # Debugging: Show the original ROI, Orange Mask, and Blue Mask
        cv2.imshow("Original ROI", roi)
        cv2.imshow("Orange Mask", orange_mask)
        cv2.imshow("Blue Mask", blue_mask)

        # Visualise the ROI
        cv2.rectangle(cv_image, (0,self.roi_y_offset), (width, self.roi_y_offset+self.roi_height), (0,255,0), 2)
        cv2.imshow("ROI Overlay", cv_image)

        rospy.loginfo(f"Orange detected: {orange_detected}, Blue detected: {blue_detected}, Turning: {self.turning}")

        # State Logic
        if not self.turning:
            if self.detecting_orange and orange_detected:
                rospy.loginfo("Orange detected. Starting gradual turn.")
                self.start_turn()
                self.detecting_orange = False
            elif not self.detecting_orange and blue_detected:
                rospy.loginfo("Blue detected. Finalizing turn.")
                self.complete_turn()
        else:
            self.continue_turn()

        cv2.waitKey(1)

    def move_straight(self):
        """Move straight at a constant speed."""
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def start_turn(self):
        """Begin the gradual 90-degree turn."""
        self.turning = True
        self.turn_start_time = rospy.Time.now()
        self.twist.linear.x = 0.15  # Slightly slower forward movement
        # Changed angular z to negative for left turn
        self.twist.angular.z = 0.2  # Begin a smooth turn to the left
        self.cmd_vel_pub.publish(self.twist)

    def continue_turn(self):
        """Continue turning smoothly."""
        if (rospy.Time.now() - self.turn_start_time) > rospy.Duration(3): # Turn for a maximum of 5 seconds
            self.complete_turn()
        else:
            self.twist.linear.x = 0.17
             # Changed angular z to negative for left turn
            self.twist.angular.z = 0.2
            self.cmd_vel_pub.publish(self.twist)

    def complete_turn(self):
        """Complete the 90-degree turn and return to straight movement."""
        self.turning = False
        self.detecting_orange = True  # Reset to detect the next orange line
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = CornerNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
