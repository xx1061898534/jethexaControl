#!/usr/bin/env python3
# coding: utf-8

import time
import rospy
from geometry_msgs.msg import Twist
from jethexa_controller_interfaces.msg import Traveling

class jetHexaBasicMotion:
    def __init__(self):
        # Initialize control node
        rospy.init_node("jethexa_basic_motion_no_subscriber", anonymous=True, log_level=rospy.INFO)

        # Build publishers
        self.traveling_pub = rospy.Publisher('/jethexa_controller/traveling', Traveling, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def forward(self, step):
        msg = Traveling()
        msg.gait = 1
        msg.stride = 40.0
        msg.height = 15.0
        msg.direction = 0
        msg.rotation = 0
        msg.time = 0.8
        msg.steps = step
        msg.relative_height = False
        msg.interrupt = False
        self.traveling_pub.publish(msg)
        print("sending")

    def horizontal_left(self, step):
        msg = Traveling()
        msg.gait = 2
        msg.stride = 40.0
        msg.height = 15.0
        msg.direction = 90
        msg.rotation = 0.0
        msg.time = 0.8
        msg.steps = step
        msg.relative_height = False
        msg.interrupt = False
        self.traveling_pub.publish(msg)

    def horizontal_right(self, step):
        msg = Traveling()
        msg.gait = 2
        msg.stride = 40.0
        msg.height = 15.0
        msg.direction = -90
        msg.rotation = 0.0
        msg.time = 0.8
        msg.steps = step
        msg.relative_height = False
        msg.interrupt = False
        self.traveling_pub.publish(msg)

    def reverse(self, step):
        msg = Traveling()
        msg.gait = 2
        msg.stride = 40.0
        msg.height = 15.0
        msg.direction = 180
        msg.rotation = 0.0
        msg.time = 0.8
        msg.steps = step
        msg.relative_height = False
        msg.interrupt = False
        self.traveling_pub.publish(msg)

    def rotation(self, rotation, step):
        msg = Traveling()
        msg.gait = 1
        msg.stride = 40.0
        msg.height = 15.0
        msg.direction = 0
        msg.rotation = rotation
        msg.time = 0.7
        msg.steps = step
        msg.relative_height = False
        msg.interrupt = False
        self.traveling_pub.publish(msg)

    def stop(self):
        msg = Traveling()
        msg.gait = 0
        msg.stride = 0.0
        msg.height = 0.0
        msg.direction = 0
        msg.rotation = 0.0
        msg.time = 0.8
        msg.steps = 1
        msg.relative_height = False
        msg.interrupt = False
        self.traveling_pub.publish(msg)

    def cmd_vel_Publisher(self, linear_x, linear_y, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        rospy.sleep(1)
        self.vel_pub.publish(msg)
    def execute_motion_sequence(self):
        self.start_recording()  # Start recording
        rospy.sleep(1)
        self.forward(0)
        rospy.sleep(10)  # Simulate some process
        self.rotation(1, 0)
        rospy.sleep(21)
        self.forward(0)
        rospy.sleep(30)  # Simulate some process
        self.rotation(1, 0)
        rospy.sleep(21)
        self.forward(0)
        rospy.sleep(10)  # Simulate some process
        self.rotation(1, 0)
        rospy.sleep(21)
        self.forward(0)
        rospy.sleep(30)  # Simulate some process
        self.rotation(1, 0)
        rospy.sleep(21)
        self.stop()
        rospy.sleep(1)
        self.stop_recording()  # Stop recording
   
    def move_in_triangle(self):
        rospy.loginfo("Starting triangular movement")
        # Move forward
        self.cmd_vel_Publisher(linear_x=0.05, linear_y=0.0, angular_z=0.0)
        rospy.sleep(10)  # Adjust sleep time based on side length

        # Rotate 120 degrees (2π/3 radians) for the triangle
        self.cmd_vel_Publisher(linear_x=-0.025, linear_y=0.05, angular_z=0)
        rospy.sleep(20)  # Adjust sleep time based on angular speed
        self.cmd_vel_Publisher(linear_x=0, linear_y=-0.05, angular_z=0)
        rospy.sleep(20)  # Adjust sleep time based on angular speed
        self.cmd_vel_Publisher(linear_x=0.0, linear_y=0.0, angular_z=0)
        self.stop()
        rospy.loginfo("Finished triangular movement")

if __name__ == "__main__":
    robot = jetHexaBasicMotion()
    rospy.sleep(1)
    robot.move_in_triangle()
