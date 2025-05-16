#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time
import math

class RobotNavigator:
    def __init__(self):
        rospy.init_node('robot_navigator', anonymous=True)

        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for IMU data
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Variables to control movement
        self.twist = Twist()
        self.imu_data = None
        self.turning = False

        # Movement duration and speed
        self.move_duration = 5  # seconds
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.2  # rad/s (smooth turning speed)

        # Start by moving straight
        self.move_straight()

    def imu_callback(self, msg):
        """Callback function to subscribe IMU data."""
        self.imu_data = msg

    def move_straight(self):
        """Move straight for a certain duration."""
        self.twist.linear.x = self.linear_speed  # Move at 0.2 m/s
        self.twist.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Moving straight")
        time.sleep(self.move_duration)
        self.start_turn()

    def start_turn(self):
        """Start turning smoothly while moving forward until 90-degree left turn is detected."""
        if self.imu_data is None:
            rospy.logwarn("IMU data not received yet.")
            return

        # Get the initial orientation (roll, pitch, yaw) from IMU data
        initial_orientation = self.imu_data.orientation
        initial_yaw = self.get_yaw(initial_orientation)

        # Start moving straight and turning
        self.twist.linear.x = self.linear_speed  # Move forward
        self.twist.angular.z = self.angular_speed  # Turn at 0.2 rad/s (left turn)
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Turning left while moving forward")

        # Wait until the robot has turned approximately 90 degrees
        while not rospy.is_shutdown():
            if self.imu_data:
                current_orientation = self.imu_data.orientation
                current_yaw = self.get_yaw(current_orientation)
                # Check if the robot has turned 90 degrees
                if abs(current_yaw - initial_yaw) >= math.radians(90):  # 90 degrees in radians
                    rospy.loginfo("Turned 90 degrees")
                    break

        # Stop turning and move straight
        self.twist.angular.z = 0.0  # Stop turning
        self.cmd_vel_pub.publish(self.twist)

        # After turn, move straight again
        time.sleep(1)  # Wait 1 second before moving straight again
        self.move_straight()

    def get_yaw(self, orientation):
        """Convert quaternion to yaw (z-axis rotation)."""
        # Extract quaternion components
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        # Convert quaternion to Euler angles (yaw, pitch, roll)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = RobotNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass

